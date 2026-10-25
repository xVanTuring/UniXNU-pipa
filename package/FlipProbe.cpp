// SPDX-License-Identifier: BSD-2-Clause-Patent
// P1 of the zero-copy scanout plan: get the display to fetch memory we control. `pipa-flip`:
//   0  (default, no argument) off - nothing maps, nothing writes, no thread starts
//   1  P1a only: the two planes' address and stride written back unchanged (passed on the tablet 2026-10-25: writes are safe)
//   2  P1c: the SMMU gate. The display's stream gets a translation context of our own whose table describes the whole 32-bit
//      input space with 2 MiB blocks - identity over this board's DRAM, our own black buffer everywhere else - and the gate
//      enables it. The picture must not change; the stream is translated afterwards, which is what P2 needs to reach the
//      compositor's surfaces (all of them above 4 GiB).
//   3  P1b: restore, red in our own 7424-stride buffer for 15 s, green in the second one for 15 s, restore. Separate from 2 so a
//      gate round never carries the flip's 21 MB contiguous searches and the zone_gc they cause.
//
// Nothing here reads a display controller register: reads to 0xae01000 produced an unrecoverable SError (see MdssProbe.cpp),
// while the writes below were proven safe on the tablet. So:
//   - SRC_XY and SRC_SIZE are left alone. Both planes keep taking their 900-wide slice of one 1800-wide row through SRC_XY
//     (x=0 and x=900), so both planes are given the SAME base address of one 1800-wide buffer and the halves stay consistent.
//   - Only SRC0_ADDR and SRC_YSTRIDE0 change: 0x9c000000 / 7200 to our buffer / 7424.
//   - P1b's buffer has to be BELOW 4 GiB: SRC0_ADDR is a 32-bit register (upstream dpu_hw_sspp.c writes it as one, with no
//     high-address companion) and the display's SMMU bank passes addresses through (SCTLR.M=0), so the register holds the
//     physical address itself. This kernel's memory starts at 0x10007c000 (measured, HANDOFF-pipa.md 10.13), which leaves no
//     room at all below 4 GiB, so 21,381,120 bytes there do not exist and the log says so.
//   - P1c is the answer to that: to reach higher DRAM the display has to be translated. The apps SMMU already routes SID 0x820
//     (mask 0x402) to context bank 4 with type=translation (P0: SMR 0x84020820, S2CR t0 b4) and only the bank's SCTLR.M=0
//     keeps it a pass-through. SMR/S2CR are only read and verified, never written - the routing is already what we need and
//     changing it would touch every stream that shares the mask.
//   - The gate's table is 2048 2 MiB BLOCK descriptors (one level-1 table, four level-2, 20 KiB total), so EVERY address the
//     DPU can name has a descriptor. That is the lesson of the hang: with only the frame buffer mapped, a fetch anywhere else
//     is a translation fault, and on this interconnect that was enough to wedge the machine (no panic, no 10 s restart, USB
//     gone). Blocks over this board's DRAM are identity; everything else - below 0x80000000 where there is no DRAM at all,
//     the gap between the banks, and a 256 MiB hole kept for P2's surface IOVAs - points at one 2 MiB black buffer of ours, so
//     a stray fetch reads black instead of faulting. See the constants below for the exact ranges.
//   - The DPU and the SMMU's table walker are not behind the CPU caches, so every buffer and table page is written through
//     `dc civac` and `dsb sy` before it is published.
//
// Display controller registers written (offsets from 0xae01000, upstream dpu_hw_sspp.c / dpu_hw_ctl.c):
//   VIG0 + 0x24 SRC_YSTRIDE0, VIG0 + 0x14 SRC0_ADDR, VIG1 + 0x24, VIG1 + 0x14, then CTL0 + 0x18 CTL_FLUSH = 0x3
// VIG0 is flush bit 0, VIG1 bit 1 (dpu_hw_ctl_update_pending_flush_sspp), and in video mode dpu_hw_ctl_trigger_flush is one
// store to CTL_FLUSH; the pending values latch at the next frame boundary, so no CTL_START is needed.
//
// Safety: P1a proved these writes do not fault. Each stage arms an independent watchdog thread with a deadline (60 s for the
// flip, 10 s for the gate); if the stage has not reported itself done by then, the watchdog writes the display's address and
// stride back, or the SMMU bank back to the firmware's image. Buffers and tables are never freed (the DPU may still be reading
// them at any moment).
#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IODeviceMemory.h>
#include <pexpert/pexpert.h>
#include <kern/thread.h>
#include <kern/task.h>
#include <kern/clock.h>
#include <libkern/c++/OSSharedPtr.h>
#include <stdint.h>
#include <string.h>
#include "ArmSmmu4K.h"
#include "ArmSmmu4KContext.h"
#include "ArmSmmu4KProgram.h"

extern "C" {
// The apps SMMU's window as MdssProbe mapped it (0 until its first round has mapped it). FlipProbe takes that mapping for the
// gate instead of making a second one of the same 1 MiB; the SMMU reads it already does every 50 s are the proof it works.
extern "C" uintptr_t native_mdss_smmu_window(void);
// For the record only: the display driver's copy of the console base, printed next to the value written.
extern volatile uint64_t native_legacy_video_base;
// MdssProbe.cpp's first round prints the P0 verdict; this test waits for it so the log carries that answer whatever the write
// turns out to do.
extern volatile uint32_t native_mdss_step;
// The physical range XNU manages (GpuProbe.cpp uses the same two), which is what decides whether a run below 4 GiB exists.
extern uint64_t vm_first_phys, vm_last_phys;
}

enum {
    kRowStatus, kRowBuffers, kRowRestore, kRowRed, kRowGreen, kRowRestored, kRowWatchdog, kRowSkipped,
    kRowGateMap, kRowGateStep, kRowGatePre, kRowGateOn, kRowGateState, kRowGateFault, kRowGateOff, kRowGateSkip, kRows
};
extern "C" {
char native_flip_rows[kRows][120];
volatile uint32_t native_flip_changes;
volatile uint32_t native_flip_offered;
}

namespace {

constexpr uint64_t kMdpBase = 0xae01000, kMdpLength = 0x8f000;
constexpr uint32_t kVig0 = 0x4000, kVig1 = 0x6000, kSrc0Addr = 0x14, kSrcYStride0 = 0x24;
constexpr uint32_t kCtl0 = 0x1000, kCtlFlush = 0x18;
constexpr uint32_t kHomeBase = 0x9c000000, kHomeStride = 7200;   // what the DPU holds now (PE_Video, and the P0 read)
constexpr uint32_t kFlipStride = 7424;                           // 1800 pixels at 4 bytes, rounded to a 64-byte line
constexpr uint32_t kFlushVig0 = 1u << 0, kFlushVig1 = 1u << 1;
constexpr uint64_t kBufferBytes = uint64_t(kFlipStride) * 2880;  // 21,381,120
// The scanout memory is BGRA8888 byte for byte, as the compositor's surfaces are (the copy path writes it unchanged).
constexpr uint32_t kRed = 0xffff0000u, kGreen = 0xff00ff00u;
constexpr unsigned kColourMs = 15000, kFlipWatchdogMs = 60000;
constexpr unsigned kFramesWriteBack = 30, kFramesFlip = 40;
constexpr unsigned kWaitFramesMs = 150000, kWaitVerdictMs = 5000;

// The apps SMMU: global space 0, then one page per context bank (the kit and NativeAdreno use the same layout).
constexpr uint64_t kSmmuBase = 0x15000000, kSmmuLength = 0x100000;
constexpr uint32_t kSmmuGfsr = 0x048, kSmmuIrqCtrl = 0x060;
constexpr uint32_t kSmmuSmr = 0x800, kSmmuS2cr = 0xc00;
constexpr uint32_t kSmmuTlbiAsid = 0x610, kSmmuTlbSync = 0x7f0, kSmmuTlbStatus = 0x7f4;
// P1c: the gate's table describes the WHOLE 32-bit input space with 2 MiB block descriptors, one level-1 table (four entries,
// one per GiB) and four level-2 tables of 512 blocks: 2048 blocks, 5 pages, 20 KiB. No address the DPU can name is left
// without a descriptor, so a fetch can neither fault nor reach a physical hole. Where the identity map is not identity:
//   [0x00000000, 0x80000000)  no DRAM at all (this board's bank 1 starts at 0x80000000): any fetch below that - including a
//                             plane whose SRC0_ADDR is 0 - would be an AXI decode error, which is what can hang the NoC. Black.
//   [0x80000000, 0xb7700000)  DRAM bank 1 (board.toml, from Android's /memory: 0x80000000+0x37700000); holds the firmware's
//                             frame buffer carveout at 0x9c000000. Identity.
//   [0xb7700000, 0xc0000000)  the gap between the two banks. Black.
//   [0xc0000000, 0xd0000000)  the hole kept for P2's surface IOVAs: inside DRAM bank 2, 256 MiB, and nothing the display reads
//                             lives there. Black today, repointed at surface memory when the time comes.
//   [0xd0000000, 0x100000000) DRAM bank 2's low 1 GiB. Identity.
// "Black" means every such block points at one 2 MiB buffer of ours filled with zeroes, so a stray fetch reads black rather
// than faulting. The table pool and that buffer are ordinary memory above 4 GiB (the SMMU's output width is 36 bits, which
// covers it); nothing here is allocated below 4 GiB, where this machine has no space.
constexpr uint64_t kBlockBytes = 0x200000;            // one 2 MiB block descriptor
constexpr unsigned kBlocksPerTable = 512, kBlockTables = 4, kBlocks = 2048;
constexpr uint64_t kTablePages = 1 + kBlockTables, kTableBytes = 8 * 4096;   // level 1 + four level 2, with slack
constexpr uint64_t kBank1Lo = 0x80000000, kBank1Hi = 0xb7700000;
constexpr uint64_t kBank2Lo = 0xc0000000, kHoleLo = 0xc0000000, kHoleHi = 0xd0000000, kInputLimit = 0x100000000;
// Level-2 block: bits[1:0]=01, nG (11), AF (10), inner shareable (SH=3 at 9:8), AttrIndx 0 (MAIR0 0x44 = Normal non-cacheable),
// AP[1]=1 (read/write), PXN|UXN (53,54) so nothing is executable. Output address in bits[47:21].
constexpr uint64_t kBlockAttributes = (UINT64_C(3) << 53) | (1u << 11) | (1u << 10) | (UINT64_C(3) << 8) | (1u << 6);
constexpr uint16_t kDisplaySid = 0x820;
constexpr unsigned kDisplayBank = 4, kDisplayAsid = 0x3f, kGatePollBudget = 1000;
constexpr unsigned kGateWindowMs = 20000, kGateWatchdogMs = 10000;
// The firmware's bank image as P0 read it, used only if the live read of one of them fails.
constexpr uint32_t kFirmwareCbar = 0x0001f000, kFirmwareCba2r = 1, kFirmwareSctlr = 0xe0;

// 0 = nothing armed, 1 = the flip (60 s), 2 = the SMMU gate (10 s unless gStable).
volatile unsigned gArmKind;
volatile bool gStable;
uint64_t gDeadlineUs;

OSSharedPtr<IOMemoryMap> gMdpMap, gWatchMap, gSmmuMap;
uintptr_t gMdp, gWatchMdp, gSmmu;

void put(unsigned row, const char *text) {
    if (row >= kRows || !strncmp(native_flip_rows[row], text, sizeof native_flip_rows[0])) return;
    strlcpy(native_flip_rows[row], text, sizeof native_flip_rows[0]);
    native_flip_changes = native_flip_changes + 1;
}
bool mapWindow(uint64_t physical, uint64_t length, OSSharedPtr<IOMemoryMap> &map, uintptr_t &virtualAddress) {
    IODeviceMemory *memory = IODeviceMemory::withRange(physical, length);
    if (!memory) return false;
    map = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
    memory->release();
    virtualAddress = map ? static_cast<uintptr_t>(map->getVirtualAddress()) : 0;
    return virtualAddress != 0;
}
void store(uintptr_t device, uint32_t offset, uint32_t value) {
    *reinterpret_cast<volatile uint32_t *>(device + offset) = value;
}
uint32_t load(uintptr_t device, uint32_t offset) {
    return *reinterpret_cast<volatile uint32_t *>(device + offset);
}
uint64_t uptimeUs() {
    uint64_t nanoseconds = 0;
    absolutetime_to_nanoseconds(mach_absolute_time(), &nanoseconds);
    return nanoseconds / 1000;
}
// Seconds since boot, as "t=63.480s", so the lines of a sequence can be placed on the boot's timeline without a register read.
void stamp(char *out, size_t cap) {
    const uint64_t us = uptimeUs();
    snprintf(out, cap, "t=%llu.%03llus", (unsigned long long)(us / 1000000), (unsigned long long)((us / 1000) % 1000));
}
// The display's master and the SMMU's table walker read DRAM behind the CPU's caches: every line written has to be cleaned to
// the point of coherency before it is published.
void cleanToPoc(const void *base, uint64_t bytes) {
    uint64_t ctr = 0;
    __asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
    const uintptr_t line = 4u << ((ctr >> 16) & 15);
    const uintptr_t end = reinterpret_cast<uintptr_t>(base) + bytes;
    for (uintptr_t at = reinterpret_cast<uintptr_t>(base) & ~(line - 1); at < end; at += line)
        __asm__ volatile("dc civac, %0" :: "r"(at) : "memory");
    __asm__ volatile("dsb sy" ::: "memory");
}
void writeScanout(uintptr_t mdp, uint32_t base, uint32_t stride) {
    store(mdp, kVig0 + kSrcYStride0, stride);
    store(mdp, kVig0 + kSrc0Addr, base);
    store(mdp, kVig1 + kSrcYStride0, stride);
    store(mdp, kVig1 + kSrc0Addr, base);
    __asm__ volatile("dsb sy" ::: "memory");   // the four pending registers reach the DPU before the flush
    store(mdp, kCtl0 + kCtlFlush, kFlushVig0 | kFlushVig1);
    __asm__ volatile("dsb sy" ::: "memory");
}

struct Buffer {IOBufferMemoryDescriptor *descriptor; unsigned char *virt; uint64_t physical, bytes;};
// One physically contiguous run below `limit`: the first and the last page must be the ends of a single segment.
bool allocateBuffer(Buffer &buffer, uint64_t bytes, uint64_t limit) {
    buffer.descriptor = nullptr;
    buffer.virt = nullptr;
    buffer.physical = 0;
    buffer.bytes = bytes;
    if (!limit || !bytes || (bytes & 0xfff) || bytes > limit) return false;
    buffer.descriptor = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
        static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous),
        bytes, (limit - 1) & ~UINT64_C(0xfff));
    if (!buffer.descriptor) return false;
    if (buffer.descriptor->prepare() != kIOReturnSuccess) {
        buffer.descriptor->release();
        buffer.descriptor = nullptr;
        return false;
    }
    IOByteCount head = 0, tail = 0;
    const addr64_t first = buffer.descriptor->getPhysicalSegment(0, &head, kIOMemoryMapperNone);
    const addr64_t last = buffer.descriptor->getPhysicalSegment(bytes - 4096, &tail, kIOMemoryMapperNone);
    buffer.virt = static_cast<unsigned char *>(buffer.descriptor->getBytesNoCopy());
    buffer.physical = first;
    if (!first || !buffer.virt || buffer.descriptor->getLength() != bytes || (first & 0xfff) ||
        last != first + bytes - 4096 || first >= limit || bytes > limit - first) {
        buffer.descriptor->complete();
        buffer.descriptor->release();
        buffer.descriptor = nullptr;
        buffer.virt = nullptr;
        buffer.physical = 0;
        return false;
    }
    return true;
}
void releaseBuffer(Buffer &buffer) {
    if (!buffer.descriptor) return;
    buffer.descriptor->complete();
    buffer.descriptor->release();
    buffer.descriptor = nullptr;
    buffer.virt = nullptr;
    buffer.physical = 0;
}
void fillColour(unsigned char *virt, uint32_t pixel) {
    volatile uint32_t *words = reinterpret_cast<volatile uint32_t *>(virt);
    const uint64_t count = kBufferBytes / sizeof(uint32_t);
    for (uint64_t i = 0; i < count; ++i) words[i] = pixel;
    cleanToPoc(virt, kBufferBytes);
}

// The P0 verdict first (so a log cut short, or ended by a panic, still answers case A/B), then the frames: N accepted swaps
// mean the compositor is presenting and the panel is being scanned by this kernel, which is when the DPU is certainly running.
bool waitForFrames(unsigned target) {
    for (unsigned waited = 0; waited < kWaitVerdictMs && native_mdss_step != 99; waited += 100) IOSleep(100);
    for (unsigned waited = 0; waited < kWaitFramesMs && native_flip_offered < target; waited += 100) IOSleep(100);
    return native_flip_offered >= target;
}

// ------------------------------------------------------------------ P1c: the SMMU gate

struct SmmuGeometry {unsigned groups, banks, pages, stride; uint64_t context0; bool usable;};
SmmuGeometry smmuGeometry() {
    SmmuGeometry geometry = {};
    const uint32_t id0 = load(gSmmu, 0x20), id1 = load(gSmmu, 0x24);
    geometry.groups = id0 & 0xff;
    geometry.banks = id1 & 0xff;
    geometry.pages = 1u << (((id1 >> 28) & 7) + 1);
    geometry.stride = (id1 >> 31) ? 0x10000u : 0x1000u;
    geometry.context0 = (uint64_t)geometry.pages * geometry.stride;
    if (geometry.groups > 128) geometry.groups = 128;   // the firmware's reliably accessible groups
    geometry.usable = geometry.groups && geometry.banks > kDisplayBank && geometry.pages &&
                      geometry.context0 + (uint64_t)geometry.banks * geometry.stride <= kSmmuLength;
    return geometry;
}
uint32_t contextAt(const SmmuGeometry &geometry, unsigned bank) {
    return (uint32_t)(geometry.context0 + (uint64_t)bank * geometry.stride);
}
// CBAR and CBA2R sit in the first register page: CBAR at stride + 4*bank, CBA2R at stride + 0x800 + 4*bank.
uint32_t cbarAt(const SmmuGeometry &geometry, unsigned bank) {
    return (uint32_t)(geometry.stride + 4 * bank);
}
uint32_t cba2rAt(const SmmuGeometry &geometry, unsigned bank) {
    return (uint32_t)(geometry.stride + 0x800 + 4 * bank);
}
uint32_t bankReg(const SmmuGeometry &geometry, unsigned bank, uint32_t offset) {
    return load(gSmmu, contextAt(geometry, bank) + offset);
}
// Only this ASID's entries: the GPU's (1, 2) and the UFS's stay valid. The kit's own synchronization writes inside the bank's
// page, which this SMMU treats as reserved, so the global TLBIASID/TLBSYNC pair is issued here as well.
void tlbiAsid(unsigned asid) {
    store(gSmmu, kSmmuTlbiAsid, asid);
    store(gSmmu, kSmmuTlbSync, 0xffffffffu);
    for (unsigned poll = 0; poll < 1000; ++poll) {
        if (!(load(gSmmu, kSmmuTlbStatus) & 1)) break;
        IODelay(1);
    }
    __asm__ volatile("dsb sy" ::: "memory");
}

struct Gate {
    Buffer tables;
    ArmSmmu4KConfig config;
    ArmSmmu4KResult built;
    ArmSmmu4KContextConfig context;
    ArmSmmu4KProgramResult program;
    unsigned slot;
    uint32_t savedCbar, savedCba2r, savedSctlr;
    Buffer black;
    uint64_t blackPhysical;
    bool ready, up;
};
Gate gGate;

int gateRead32(void *, size_t offset, uint32_t *value) {
    if (!value || (offset & 3) || offset + 4 > kSmmuLength) return -1;
    *value = load(gSmmu, (uint32_t)offset);
    return 0;
}
int gateRead64(void *, size_t offset, uint64_t *value) {
    if (!value || (offset & 7) || offset + 8 > kSmmuLength) return -1;
    *value = *reinterpret_cast<volatile uint64_t *>(gSmmu + offset);
    return 0;
}
int gateWrite32(void *, size_t offset, uint32_t value) {
    if ((offset & 3) || offset + 4 > kSmmuLength) return -1;
    store(gSmmu, (uint32_t)offset, value);
    return 0;
}
int gateWrite64(void *, size_t offset, uint64_t value) {
    if ((offset & 7) || offset + 8 > kSmmuLength) return -1;
    *reinterpret_cast<volatile uint64_t *>(gSmmu + offset) = value;
    return 0;
}
int gateBarrier(void *) {
    __asm__ volatile("dsb sy" ::: "memory");
    return 0;
}
const ArmSmmu4KIo kGateIo = {nullptr, kSmmuLength, gateRead32, gateRead64, gateWrite32, gateWrite64, gateBarrier};

// The firmware's bank image written back, M=0 first: with the bank disabled the stream is back to the firmware's pass-through
// whatever else is left in the bank. Then CBAR/CBA2R/SCTLR, with only this ASID's TLB entries dropped. Idempotent.
void writeBankImage(uint32_t cbar, uint32_t cba2r, uint32_t sctlr) {
    const SmmuGeometry geometry = smmuGeometry();
    store(gSmmu, contextAt(geometry, kDisplayBank), 0);
    __asm__ volatile("dsb sy" ::: "memory");
    tlbiAsid(kDisplayAsid);
    store(gSmmu, cba2rAt(geometry, kDisplayBank), cba2r);
    store(gSmmu, cbarAt(geometry, kDisplayBank), cbar);
    store(gSmmu, contextAt(geometry, kDisplayBank), sctlr);
    __asm__ volatile("dsb sy" ::: "memory");
    tlbiAsid(kDisplayAsid);
}
void gateRestore(const char *why) {
    if (!gGate.up) return;
    writeBankImage(gGate.savedCbar, gGate.savedCba2r, gGate.savedSctlr);
    gGate.up = false;
    gArmKind = 0;
    char line[120];
    snprintf(line, sizeof line, "smmu restored (%s): sctlr %08x cbar %08x cba2r %x", why, gGate.savedSctlr,
             gGate.savedCbar, gGate.savedCba2r);
    put(kRowGateOff, line);
}

// One row per milestone. put() prints a row only when its text changes, so the log carries every step the gate reached and
// stops at the one it did not come back from.
void gateStep(unsigned step, const char *what) {
    char line[120];
    snprintf(line, sizeof line, "gate step %u %s", step, what);
    put(kRowGateStep, line);
}
bool gateFail(unsigned step, const char *why) {
    gateStep(step, why);
    char line[120];
    snprintf(line, sizeof line, "smmu gate skipped at step %u", step);
    put(kRowGateSkip, line);
    return false;
}

// The SMMU window. MdssProbe maps the same 1 MiB for its own rounds and reads it every 50 s, so its mapping is preferred: this
// thread then does no I/O mapping of its own, and none of the VM work that comes with it. A missing one is mapped here.
bool gateWindow() {
    for (unsigned waited = 0; waited < 5000 && !gSmmu; waited += 100) {
        gSmmu = native_mdss_smmu_window();
        if (!gSmmu) IOSleep(100);
    }
    const bool own = !gSmmu;
    if (own && !mapWindow(kSmmuBase, kSmmuLength, gSmmuMap, gSmmu))
        return gateFail(1, "window FAILED: no MdssProbe mapping and our own was refused");
    gateStep(1, own ? "window: our own mapping" : "window: MdssProbe's mapping");
    return true;
}

// Is this 2 MiB block covered by the identity map? A block only counts when the whole 2 MiB lies inside one DRAM bank: a block
// straddling the end of a bank would send part of its accesses into the gap. Everything else points at the black buffer.
bool blockIsIdentity(uint64_t base) {
    const uint64_t end = base + kBlockBytes;
    const bool bank1 = base >= kBank1Lo && end <= kBank1Hi;
    const bool bank2 = base >= kBank2Lo && end <= kInputLimit;
    if (!bank1 && !bank2) return false;
    if (base >= kHoleLo && base < kHoleHi) return false;   // the hole kept for surface IOVAs
    return true;
}
// Walk the table exactly as the SMMU will - level 1 indexed by bits[31:30], level 2 by bits[29:21] - and count the blocks whose
// answer is not what blockIsIdentity() says it should be. Nothing is enabled while this is not zero.
unsigned gateVerify() {
    const uint64_t *const level1 = reinterpret_cast<const uint64_t *>(gGate.tables.virt);
    unsigned bad = 0;
    for (unsigned gigabyte = 0; gigabyte < kBlockTables; ++gigabyte) {
        const uint64_t entry = level1[gigabyte];
        const uint64_t tablePhysical = entry & UINT64_C(0x0000fffffffff000);
        if ((entry & 3) != 3 || tablePhysical < gGate.tables.physical ||
            tablePhysical >= gGate.tables.physical + kTablePages * 4096) {
            ++bad;
            continue;
        }
        const uint64_t *const level2 =
            reinterpret_cast<const uint64_t *>(gGate.tables.virt + (tablePhysical - gGate.tables.physical));
        for (unsigned index = 0; index < kBlocksPerTable; ++index) {
            const uint64_t base = ((uint64_t)gigabyte << 30) + ((uint64_t)index << 21);
            const uint64_t want = blockIsIdentity(base) ? base : gGate.blackPhysical;
            const uint64_t block = level2[index];
            if ((block & 3) != 1 || ((block ^ want) & UINT64_C(0x0000ffffffe00000))) ++bad;
        }
    }
    return bad;
}

// Everything that does not touch the display's bank: the window, the geometry, the stream's routing (read only), the image to
// restore, the cold-start restore itself, and our own tables. This runs at thread start, while the machine is still healthy;
// the frame wait and P1b's 21 MB allocations come after it.
bool gateSetup() {
    char line[120];
    if (!gateWindow()) return false;
    const SmmuGeometry geometry = smmuGeometry();
    snprintf(line, sizeof line, "geometry: groups %u banks %u pages %u ctxt %llx", geometry.groups, geometry.banks,
             geometry.pages, (unsigned long long)geometry.context0);
    gateStep(2, line);
    if (!geometry.usable) return gateFail(2, "FAILED: unsupported SMMU geometry");
    // Exactly one stream-match group may cover the display's SID, and it must already translate to bank 4: that is what P0
    // read, and nothing here writes SMR or S2CR.
    unsigned matches = 0, slot = 0;
    for (unsigned i = 0; i < geometry.groups; ++i) {
        const uint32_t smr = load(gSmmu, kSmmuSmr + 4 * i);
        if (!(smr & 0x80000000u)) continue;
        const uint32_t mask = (smr >> 16) & 0x7fff;
        if (((smr ^ kDisplaySid) & 0xffff) & ~mask) continue;
        matches = matches + 1;
        slot = i;
    }
    const uint32_t s2cr = matches == 1 ? load(gSmmu, kSmmuS2cr + 4 * slot) : 0;
    const unsigned type = (s2cr >> 16) & 3, bank = s2cr & 0xff;
    snprintf(line, sizeof line, "stream: smr #%u matches %u s2cr t%u b%u", slot, matches, type, bank);
    gateStep(3, line);
    if (matches != 1 || type != 0 || bank != kDisplayBank)
        return gateFail(3, "FAILED: the display SID is not one translation entry to bank 4");
    const uint32_t bankSctlr = bankReg(geometry, kDisplayBank, 0x00);
    if (bankSctlr & 1) {
        // A bank left translating is not ours to build on: put the firmware's image back (M=0 first) so the stream returns to
        // pass-through, and leave the gate for the next round.
        writeBankImage(kFirmwareCbar, kFirmwareCba2r, kFirmwareSctlr);
        gateStep(3, "bank 4 was translating: firmware image written back");
        put(kRowGateSkip, "smmu gate skipped at step 3: bank 4 already translated, healed to M=0");
        return false;
    }
    gGate.slot = slot;
    gGate.savedCbar = load(gSmmu, cbarAt(geometry, kDisplayBank));
    gGate.savedCba2r = load(gSmmu, cba2rAt(geometry, kDisplayBank));
    gGate.savedSctlr = bankSctlr;
    if (gGate.savedCbar == 0 || gGate.savedCbar == 0xffffffffu) gGate.savedCbar = kFirmwareCbar;
    if (gGate.savedCba2r == 0 || gGate.savedCba2r == 0xffffffffu) gGate.savedCba2r = kFirmwareCba2r;
    if (gGate.savedSctlr == 0xffffffffu) gGate.savedSctlr = kFirmwareSctlr;
    // Fault interrupts: nothing in this kernel handles the apps SMMU's, and a level-triggered one that nobody clears would
    // storm. Report what is there and mask it if it is on. A pending global fault is cleared so the window starts at zero.
    const uint32_t irqCtrl = load(gSmmu, kSmmuIrqCtrl);
    const uint32_t gfsr = load(gSmmu, kSmmuGfsr);
    if (irqCtrl) store(gSmmu, kSmmuIrqCtrl, 0);
    if (gfsr) store(gSmmu, kSmmuGfsr, gfsr);   // write one to clear
    snprintf(line, sizeof line, "smmu pre: irq_ctrl %x gfsr %x, bank %u sctlr %08x cbar %08x cba2r %x", irqCtrl, gfsr,
             kDisplayBank, gGate.savedSctlr, gGate.savedCbar, gGate.savedCba2r);
    put(kRowGatePre, line);
    // The cold-start restore the plan asks for: the firmware's image written once, before anything else is tried, so a bank
    // left translating by an earlier attempt stops here. It writes back the values just read, so a healthy boot changes nothing.
    writeBankImage(gGate.savedCbar, gGate.savedCba2r, gGate.savedSctlr);
    gateStep(4, "restore written (idempotent, M=0)");
    // Our own table pool, our own 2 MiB black buffer, and our own tables: nothing of the firmware's 0x1f000 table is reused.
    if (!allocateBuffer(gGate.tables, kTableBytes, UINT64_C(0x1000000000)))
        return gateFail(5, "FAILED: no contiguous table pool");
    // A level-2 block's output address is bits[47:21], so it must be 2 MiB aligned: take 4 MiB and use the aligned 2 MiB inside
    // it (a block descriptor cannot express a smaller alignment, and writing one would put address bits into the attribute bits).
    if (!allocateBuffer(gGate.black, 2 * kBlockBytes, UINT64_C(0x1000000000))) {
        releaseBuffer(gGate.tables);
        return gateFail(5, "FAILED: no 4 MiB buffer for the 2 MiB black map");
    }
    gGate.blackPhysical = (gGate.black.physical + kBlockBytes - 1) & ~(kBlockBytes - 1);
    if (gGate.blackPhysical + kBlockBytes > gGate.black.physical + gGate.black.bytes) {
        releaseBuffer(gGate.black);
        releaseBuffer(gGate.tables);
        return gateFail(5, "FAILED: the black buffer has no 2 MiB aligned half");
    }
    unsigned char *const blackVirt = gGate.black.virt + (gGate.blackPhysical - gGate.black.physical);
    memset(gGate.tables.virt, 0, kTableBytes);   // every slot not written below is a fault; there should be none
    memset(blackVirt, 0, kBlockBytes);           // all-zero pixels: black
    cleanToPoc(blackVirt, kBlockBytes);
    unsigned identity = 0, black = 0;
    for (unsigned gigabyte = 0; gigabyte < kBlockTables; ++gigabyte) {
        uint64_t *const level1 = reinterpret_cast<uint64_t *>(gGate.tables.virt);
        uint64_t *const level2 = reinterpret_cast<uint64_t *>(gGate.tables.virt + (gigabyte + 1) * 4096);
        level1[gigabyte] = (gGate.tables.physical + (gigabyte + 1) * 4096) | 3;   // table descriptor
        for (unsigned block = 0; block < kBlocksPerTable; ++block) {
            const uint64_t base = ((uint64_t)gigabyte << 30) + ((uint64_t)block << 21);
            const bool same = blockIsIdentity(base);
            level2[block] = (same ? base : gGate.blackPhysical) | kBlockAttributes | 1;   // bits[1:0]=01: a 2 MiB block
            if (same) ++identity; else ++black;
        }
    }
    cleanToPoc(gGate.tables.virt, kTablePages * 4096);
    snprintf(line, sizeof line, "tables: %u blocks (%u identity, %u black), %llu pages", kBlocks, identity, black,
             (unsigned long long)kTablePages);
    gateStep(5, line);
    // The check that keeps a mistake in this arithmetic from reaching the hardware: walk the table the way the SMMU will and
    // compare every block's answer with the model. One mismatch and the enable never happens.
    const unsigned mismatched = gateVerify();
    if (mismatched) {
        snprintf(line, sizeof line, "FAILED: %u of %u blocks do not match the map", mismatched, kBlocks);
        gateFail(5, line);
        releaseBuffer(gGate.black);
        releaseBuffer(gGate.tables);
        return false;
    }
    // The encoder and the programmer take this result; the tables are ours, so the result describes them.
    gGate.config = ArmSmmu4KConfig{32, 36, gGate.tables.physical, kBlocks};
    gGate.built = ArmSmmu4KResult{};
    gGate.built.root_physical = gGate.tables.physical;
    gGate.built.mair = 0xff44;
    gGate.built.table_pages = kTablePages;
    gGate.built.leaf_pages = kBlocks;   // the encoder only compares this with leaf_budget; here one leaf is one block
    gGate.built.start_level = 1;        // 32-bit input: root is level 1, two index bits, then nine
    gGate.built.upper_region = 0;
    gGate.context = ArmSmmu4KContextConfig{};
    gGate.context.lower = &gGate.config;
    gGate.context.lower_tables = &gGate.built;
    gGate.context.asid = kDisplayAsid;
    // The board's working GPU context uses the same two: coherent walks, and a fault terminates the access instead of stalling
    // the master (a stalled display fetch would never recover).
    gGate.context.coherent_walk = 1;
    gGate.context.stall_on_fault = 0;
    gGate.context.continue_after_fault = 1;
    snprintf(line, sizeof line, "smmu armed: black %llx for no-DRAM + hole %llx..%llx, asid %u",
             (unsigned long long)gGate.blackPhysical, (unsigned long long)kHoleLo, (unsigned long long)kHoleHi,
             kDisplayAsid);
    put(kRowGateMap, line);
    gGate.ready = true;
    gateStep(6, "ready: nothing armed, waiting for frames");
    return true;
}

// Program the bank with our image and turn translation on. The kit writes SCTLR=0, CBA2R/CBAR/TCR2/TCR/TTBR0/TTBR1/MAIR,
// clears the bank's fault status, drops this ASID's TLB entries, verifies the whole image while M is still 0, then writes
// SCTLR with M=1 and verifies it again.
bool gateEnable() {
    char line[120];
    gateStep(7, "enable: programming bank 4");
    const ArmSmmu4KStatus status = ArmSmmu4KProgramDisabledContext(&kGateIo, kDisplayBank, &gGate.context, kGatePollBudget,
        &gGate.program);
    if (status != ArmSmmu4KSuccess) {
        snprintf(line, sizeof line, "FAILED: status %u phase %u writes %u", (unsigned)status, gGate.program.phase,
                 gGate.program.writes);
        gateFail(7, line);
        return false;
    }
    const SmmuGeometry geometry = smmuGeometry();
    const uint32_t sctlr = bankReg(geometry, kDisplayBank, 0x00);
    const uint32_t smr = load(gSmmu, kSmmuSmr + 4 * gGate.slot);
    const uint32_t s2cr = load(gSmmu, kSmmuS2cr + 4 * gGate.slot);
    if (!(sctlr & 1)) {
        snprintf(line, sizeof line, "FAILED: sctlr %08x still M=0", sctlr);
        gateFail(7, line);
        return false;
    }
    snprintf(line, sizeof line, "smmu enabled at frame %u: smr #%u mask %x t%u b%u, sctlr %08x M=1", native_flip_offered,
             gGate.slot, (smr >> 16) & 0x7fff, (s2cr >> 16) & 3, s2cr & 0xff, sctlr);
    put(kRowGateOn, line);
    put(kRowGateState, "smmu stable, screen unchanged expected");
    snprintf(line, sizeof line, "enabled: sctlr %08x M=1, standing down the watchdog", sctlr);
    gateStep(8, line);
    gStable = true;   // the enable is verified; the watchdog stands down
    return true;
}

// The gate itself: arm, enable, watch for a window, report the fault registers, and put the bank back whatever they say.
void gateRun() {
    if (!gGate.ready) return;
    char line[120];
    // The watchdog starts counting before the first write of the bank.
    gStable = false;
    gDeadlineUs = uptimeUs() + (uint64_t)kGateWatchdogMs * 1000;
    gArmKind = 2;
    gGate.up = true;
    if (!gateEnable()) {
        gateRestore("the bank did not enable");
        return;
    }
    IOSleep(kGateWindowMs);
    const SmmuGeometry geometry = smmuGeometry();
    const uint32_t gfsr = load(gSmmu, kSmmuGfsr);
    const uint32_t fsr = bankReg(geometry, kDisplayBank, 0x58);
    const uint32_t farLo = bankReg(geometry, kDisplayBank, 0x60);
    const uint32_t farHi = bankReg(geometry, kDisplayBank, 0x64);
    snprintf(line, sizeof line, "smmu faults %s: gfsr %08x fsr %08x far %x%08x", (gfsr || fsr) ? "SEEN" : "none", gfsr,
             fsr, farHi, farLo);
    put(kRowGateFault, line);
    gateStep(9, "window closed, restoring");
    gateRestore("window closed");
}

// ------------------------------------------------------------------ P1b: our own buffer

// Red and green in our own buffers at 7424, then the firmware's buffer and stride back.
void runFlip() {
    char line[120];
    Buffer red = {}, green = {};
    // Below 4 GiB first: only that is reachable through a 32-bit SRC0_ADDR with the stream in bypass. If the machine cannot
    // give a run there, a 36-bit attempt is made anyway so the log says what it could have given instead.
    if (!allocateBuffer(red, kBufferBytes, UINT64_C(0x100000000)) ||
        !allocateBuffer(green, kBufferBytes, UINT64_C(0x100000000))) {
        releaseBuffer(red);
        releaseBuffer(green);
        Buffer wide = {};
        const bool haveWide = allocateBuffer(wide, kBufferBytes, UINT64_C(0x1000000000));
        snprintf(line, sizeof line, "no flip: no %llu B run below 4 GiB (SRC0_ADDR is 32-bit)",
                 (unsigned long long)kBufferBytes);
        put(kRowSkipped, line);
        if (haveWide)
            snprintf(line, sizeof line, "free phys %llx-%llx, a 36-bit run at %llx is out of reach",
                     (unsigned long long)vm_first_phys, (unsigned long long)vm_last_phys,
                     (unsigned long long)wide.physical);
        else
            snprintf(line, sizeof line, "free phys %llx-%llx, no 36-bit run either",
                     (unsigned long long)vm_first_phys, (unsigned long long)vm_last_phys);
        put(kRowBuffers, line);
        releaseBuffer(wide);
        return;
    }
    fillColour(red.virt, kRed);
    fillColour(green.virt, kGreen);
    snprintf(line, sizeof line, "buffers red %llx green %llx, %llu B each, below 4 GiB",
             (unsigned long long)red.physical, (unsigned long long)green.physical, (unsigned long long)kBufferBytes);
    put(kRowBuffers, line);
    gDeadlineUs = uptimeUs() + (uint64_t)kFlipWatchdogMs * 1000;
    gArmKind = 1;
    writeScanout(gMdp, kHomeBase, kHomeStride);
    char when[40];
    stamp(when, sizeof when);
    snprintf(line, sizeof line, "restore %08x/%u at frame %u %s (idempotent)", kHomeBase, kHomeStride, native_flip_offered, when);
    put(kRowRestore, line);
    IOSleep(500);
    writeScanout(gMdp, (uint32_t)red.physical, kFlipStride);
    stamp(when, sizeof when);
    snprintf(line, sizeof line, "red at frame %u %s: addr %08x stride %u", native_flip_offered, when,
             (uint32_t)red.physical, kFlipStride);
    put(kRowRed, line);
    IOSleep(kColourMs);
    writeScanout(gMdp, (uint32_t)green.physical, kFlipStride);
    stamp(when, sizeof when);
    snprintf(line, sizeof line, "green at frame %u %s: addr %08x stride %u", native_flip_offered, when,
             (uint32_t)green.physical, kFlipStride);
    put(kRowGreen, line);
    IOSleep(kColourMs);
    writeScanout(gMdp, kHomeBase, kHomeStride);
    stamp(when, sizeof when);
    snprintf(line, sizeof line, "restored at frame %u %s: %08x/%u", native_flip_offered, when, kHomeBase, kHomeStride);
    put(kRowRestored, line);
    gArmKind = 0;
}

// ------------------------------------------------------------------ stages and entry

// P1a: the two planes' address and stride written back with the values the DPU already holds, so no frame can look different.
void writeBackThread(void *, wait_result_t) {
    put(kRowStatus, "write-back armed: waiting for 30 frames from the display driver");
    if (!mapWindow(kMdpBase, kMdpLength, gMdpMap, gMdp)) {
        put(kRowSkipped, "write-back skipped: the MDSS window did not map");
        return;
    }
    if (!waitForFrames(kFramesWriteBack)) {
        char line[120];
        snprintf(line, sizeof line, "write-back skipped: not enough frames (%u offered)", native_flip_offered);
        put(kRowSkipped, line);
        return;
    }
    writeScanout(gMdp, kHomeBase, kHomeStride);
    IOSleep(200);   // an SError from an unsafe write lands here, before the success line rather than after it
    char line[120];
    snprintf(line, sizeof line, "write-back ok at frame %u: VIG0/VIG1 %08x/%u flush %x, pe %llx", native_flip_offered,
             kHomeBase, kHomeStride, kFlushVig0 | kFlushVig1, (unsigned long long)native_legacy_video_base);
    put(kRowRestore, line);
}

// P1b on its own thread: the flip's own 30 s sequence, and nothing after it.
void flipThread(void *, wait_result_t) {
    put(kRowStatus, "flip armed: waiting for 40 frames; red 15 s, green 15 s, then the SMMU gate");
    if (!mapWindow(kMdpBase, kMdpLength, gMdpMap, gMdp)) {
        put(kRowSkipped, "no flip: the MDSS window did not map");
        return;
    }
    if (!waitForFrames(kFramesFlip)) {
        char line[120];
        snprintf(line, sizeof line, "no flip: not enough frames (%u offered)", native_flip_offered);
        put(kRowSkipped, line);
        return;
    }
    runFlip();
}

// P1c on a thread of its own, so the gate can never be held up by P1b's allocations or by anything the flip leaves behind: the
// setup (window, geometry, tables) runs at once, before the frame wait and before the first 21 MB attempt.
void gateThread(void *, wait_result_t) {
    if (!gateSetup()) return;
    if (!waitForFrames(kFramesFlip)) {
        char line[120];
        snprintf(line, sizeof line, "smmu gate skipped: not enough frames (%u offered)", native_flip_offered);
        put(kRowGateSkip, line);
        return;
    }
    gateRun();
}

// Independent of both stages: from the moment a stage arms, if it has not finished by its deadline this puts the display's
// address and stride back (the flip) or the SMMU bank back to the firmware's image (the gate). It never reads a display
// controller register and never frees a buffer or a table.
void watchdogThread(void *, wait_result_t) {
    const bool haveMdp = mapWindow(kMdpBase, kMdpLength, gWatchMap, gWatchMdp);
    for (;;) {
        IOSleep(500);
        const unsigned kind = gArmKind;
        if (!kind || uptimeUs() < gDeadlineUs) continue;
        if (kind == 1) {
            if (haveMdp) {
                writeScanout(gWatchMdp, kHomeBase, kHomeStride);
                char when[40];
                stamp(when, sizeof when);
                char line[120];
                snprintf(line, sizeof line, "watchdog restored %08x/%u %s, the flip did not finish", kHomeBase, kHomeStride,
                         when);
                put(kRowWatchdog, line);
            }
            gArmKind = 0;
        } else if (!gStable) {
            gateRestore("the gate did not finish");
        }
    }
}
}  // namespace

// Called from the late probe (LegacyDisplayProbe.cpp, after native_mdss_probe). The watchdog is started first, so it exists
// before anything can arm it.
extern "C" void native_flip_probe(void) {
    static bool started;
    if (started) return;
    // Off unless asked for: P1c's SMMU gate hung the device once it turned translation on (2026-10-25, the tablet stopped
    // answering USB and had to be power cycled), and a round that hangs costs a human at the tablet. 1 = the P1a write-back,
    // 2 = the P1c gate (the 2 MiB block map), 3 = P1b's red/green flip. The gate and the flip are separate modes so a gate round
    // never carries the flip's 21 MB contiguous searches and the zone_gc they cause. `pf` is the same argument in four fewer
    // characters, for a command line with almost no room left.
    unsigned mode = 0;
    if (!PE_parse_boot_argn("pipa-flip", &mode, sizeof mode)) PE_parse_boot_argn("pf", &mode, sizeof mode);
    if (!mode) return;
    if (mode > 3) mode = 3;
    started = true;
    thread_t thread = nullptr;
    if (mode == 1) {
        if (kernel_thread_start(writeBackThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
        return;
    }
    if (kernel_thread_start(watchdogThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    if (mode == 2 && kernel_thread_start(gateThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    if (mode == 3 && kernel_thread_start(flipThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
}

// Called by the display driver's offer path (LegacyDisplayProbe.cpp) for every frame it hands on.
extern "C" void native_flip_frame_offered(void) {
    native_flip_offered = native_flip_offered + 1;
}
