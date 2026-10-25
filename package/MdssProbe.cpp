// SPDX-License-Identifier: BSD-2-Clause-Patent
// The P0 read-only map of the zero-copy scanout plan (HANDOFF-pipa-zerocopy.md) that XNU can make safely: the apps SMMU's
// display streams (case A/B/C), the frame buffer the firmware left, and the compositor's surfaces. It reads no display
// controller register; the reason is the panic of 2026-10-25 and is set out below.
//
// Why the MDSS block (0xae01000) is not read here. The first build of this probe read MDP_HW_VERSION at 0xae01000+0x0 six
// seconds after the late probe and the kernel panicked with "Unhandled implementation specific error". That string is
// osfmk/arm64/sleh.c:399, reached from sleh_serror (:2261) through arm64_platform_error (:446, PLAT_ERR_SRC_ASYNC) when no
// platform error handler takes the error: an SError, i.e. the interconnect reported a failed transaction. This kernel has no
// SError recovery, so one bad read is fatal. An SError is asynchronous — it can arrive after the load retired, on any CPU —
// so no check, no per-block mapping and no heartbeat before the read can make such an access safe: the round before the panic
// read the eight SSPPs and both CTLs and then died, the last one died on the very first read. The block is also the one the
// kernel cannot vouch for: nothing in this kernel votes MDSS's GDSC or its clocks, so whether its registers answer depends on
// what the bootloader left, and two rounds with the same image gave two different answers.
//
// The MDSS facts therefore belong in the UEFI payload, where the firmware's own display programming is intact. What that
// reader has to dump once it has an output channel of its own (payload-adt.log is written by the host script, not by the
// payload, so this needs an ESP file or a DRAM/IMEM slot), with the order this probe used, HW_VERSION first and every block
// abandoned if it does not answer:
//   mdp 0xae01000 + 0x000 HW_VERSION (SDE 6.0 answers 0x60000000)
//   VIG0..3 0x4000/0x6000/0x8000/0xa000, DMA0..3 0x24000/0x26000/0x28000/0x2a000:
//     +0x00 SRC_SIZE (height-1 << 16 | width-1), +0x08 SRC_XY, +0x14 SRC0_ADDR, +0x24 SRC_YSTRIDE0 (low half, 7200),
//     +0x30 SRC_FORMAT
//   CTL0/CTL1 0x1000/0x1200: +0x00 CTL_LAYER(0), +0x04 CTL_LAYER(1), +0x18 CTL_FLUSH, +0xf4 CTL_INTF_ACTIVE
//   INTF1/INTF2 0x6a800/0x6b000: +0x000 TIMING_ENGINE_EN, +0x008 HSYNC_CTL, +0x00c VSYNC_PERIOD_F0,
//     +0x0ac FRAME_COUNT, +0x0b0 LINE_COUNT, +0x26c STATUS; FRAME_COUNT again 1 s later gives the refresh rate
//   PP0/PP1 0x70000/0x70800: +0x25c INTF_MUX
//
// What is read here: the apps SMMU at 0x15000000, which this very kernel proves live — the touch probe (pipa-touch=5) maps
// the same 0x100000 window and rewrites an SMR/S2CR in it every boot (TouchProbe.cpp:421-427 through SmmuBypass.h), and the
// UFS driver's DMA runs through it. Plus PE_Video and the two published frame buffer mappings (plain memory), and the frame
// descriptors the display offers (a pointer compare per frame, a segment walk once per new one).
//
// Rows (at most 119 characters each) go to the kernel log (BootRamoops.c, "mdss:") when their text changes, the decision
// first; a round every 50 s refreshes the presented frame rate and the surface count.
#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOPlatformExpert.h>
#include <pexpert/pexpert.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <mach/kmod.h>
#include <libkern/c++/OSSharedPtr.h>
#include <stdint.h>
#include <string.h>

extern "C" {
// LegacyDisplayProbe.cpp (the display driver's mapping of the frame buffer, and PE_Video's base and extent) and
// NativeGOPPresenter.cpp (its own mapping of the same memory) publish these. native_gop_bytes is not the frame buffer's
// size: the presenter adds to it for every byte it copies (NativeGOPPresenter.cpp:131), so it reads 0 on the GPU hand-off.
extern volatile uint64_t native_gop_physical;
extern volatile uint64_t native_legacy_gop_physical, native_legacy_gop_bytes;
}

// Row order: 0 the decision, 1 the SMMU's global registers, 2-5 the two display SIDs, 6-9 the whole stream-match table,
// 10-11 the PE_Video fields, 12 the published frame buffer addresses, 13 the compositor's surfaces, 14 why there is no
// scanout-side data. The rest stay empty (the log copies 31 rows, as BootRamoops.c declares them).
enum {
    kRowCase, kRowSmmu, kRowSid, kRowSid2 = kRowSid + 2, kRowSmr = kRowSid2 + 2,
    kRowVideo = kRowSmr + 4, kRowVideo2, kRowGop, kRowSurfaces, kRowState, kRows
};
extern "C" {
char native_mdss_rows[31][120];
volatile uint32_t native_mdss_changes;
volatile uint32_t native_mdss_step;
}

namespace {

constexpr uint64_t kSmmuBase = 0x15000000, kSmmuLength = 0x100000;
constexpr uint32_t kSmmuGbpa = 0x000, kSmmuIdr0 = 0x020, kSmmuIdr1 = 0x024, kSmmuIdr2 = 0x028;
constexpr uint32_t kSmmuGfsr = 0x048, kSmmuSmr = 0x800, kSmmuS2cr = 0xc00, kSmmuCbar = 0x1000, kSmmuCba2r = 0x1800;

// mdss's stream IDs: the non-secure display context and the secure one (the latter is only read, never taken over).
constexpr uint16_t kSidUnsec = 0x820, kSidSec = 0x840;

OSSharedPtr<IOMemoryMap> gSmmuMap;
uintptr_t gSmmu;
bool gSmmuOk;

// A row is written only when its text changed: the log prints the rows whose text differs from the previous pass.
void put(unsigned row, const char *text) {
    if (row >= kRows || !strncmp(native_mdss_rows[row], text, sizeof native_mdss_rows[0])) return;
    strlcpy(native_mdss_rows[row], text, sizeof native_mdss_rows[0]);
    native_mdss_changes = native_mdss_changes + 1;
}
bool mapWindow(uint64_t physical, uint64_t length, OSSharedPtr<IOMemoryMap> &map, uintptr_t &virtualAddress) {
    IODeviceMemory *memory = IODeviceMemory::withRange(physical, length);
    if (!memory) return false;
    map = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
    memory->release();
    virtualAddress = map ? static_cast<uintptr_t>(map->getVirtualAddress()) : 0;
    return virtualAddress != 0;
}
// A register that reads back all zeroes or all ones is a block that is not answering.
bool answers(uint32_t value) {return value != 0 && value != 0xffffffffu;}
uint32_t smmuRead(uint32_t offset) {return *reinterpret_cast<volatile uint32_t *>(gSmmu + offset);}
uint64_t uptimeUs() {
    uint64_t nanoseconds = 0;
    absolutetime_to_nanoseconds(mach_absolute_time(), &nanoseconds);
    return nanoseconds / 1000;
}

struct SmmuGeometry {unsigned groups, banks, pages; bool usable;};
SmmuGeometry smmuGeometry() {
    SmmuGeometry geometry = {};
    if (!answers(smmuRead(kSmmuIdr0))) return geometry;
    const uint32_t idr1 = smmuRead(kSmmuIdr1);
    geometry.groups = smmuRead(kSmmuIdr0) & 0xff;
    geometry.banks = idr1 & 0xff;
    geometry.pages = 1u << (((idr1 >> 28) & 7) + 1);
    if (geometry.groups > 128) geometry.groups = 128;   // the firmware's reliably accessible groups (SmmuBypass.h)
    geometry.usable = geometry.groups && geometry.banks && geometry.banks <= 128 &&
                      (geometry.pages + geometry.banks) * 4096u <= kSmmuLength;
    return geometry;
}

struct SidView {
    unsigned matches, index, bank, mask, type;
    uint32_t smr, s2cr, cbar, cba2r, sctlr;
    bool found, bankRead;
};
// Every SMR whose stream-match covers this SID. SMR's low half is the ID and bits 16-30 the mask that is not compared;
// S2CR's bits 16-17 are the type (0 translation, 1 bypass, 2 fault) and bits 0-7 the context bank.
SidView analyseSid(uint16_t sid, const SmmuGeometry &geometry) {
    SidView view = {};
    view.index = 0xffffffffu;
    view.type = 3;
    for (unsigned i = 0; i < geometry.groups; ++i) {
        const uint32_t smr = smmuRead(kSmmuSmr + 4 * i);
        if (!(smr & 0x80000000u)) continue;
        const uint32_t mask = (smr >> 16) & 0x7fff;
        if (((smr ^ sid) & 0xffff) & ~mask) continue;
        if (view.found) {view.matches = view.matches + 1; continue;}
        view.found = true;
        view.matches = 1;
        view.index = i;
        view.mask = mask;
        view.smr = smr;
        view.s2cr = smmuRead(kSmmuS2cr + 4 * i);
        view.type = (view.s2cr >> 16) & 3;
        view.bank = view.s2cr & 0xff;
        if (view.bank < geometry.banks) {
            view.cbar = smmuRead(kSmmuCbar + 4 * view.bank);
            view.cba2r = smmuRead(kSmmuCba2r + 4 * view.bank);
            view.sctlr = smmuRead((geometry.pages + view.bank) * 4096u);
            view.bankRead = true;
        }
    }
    return view;
}
// A (bypass or identity: nothing to do), B (a page table: the stream has to be taken over) or C (a masked match that shares
// its context bank with other streams). An unmatched stream is read from sCR0's USFCFG (bit 10): clear means it passes
// untranslated.
const char *sidVerdict(const SidView &view, uint32_t gbpa) {
    if (view.matches > 1) return "C conflict";
    if (!view.found) return (gbpa & (1u << 10)) ? "none, faults" : "none, bypass";
    if (view.type == 1) return view.mask ? "C mask, bypass" : "A bypass";
    if (view.type == 2) return "fault";
    if (view.type != 0) return "unknown";
    const bool translating = view.bankRead && (view.sctlr & 1);
    if (view.mask) return translating ? "C mask, page table" : "C mask, bank off";
    return translating ? "B page table" : "A bank off";
}
void rowsSid(uint16_t sid, const SidView &view, const SmmuGeometry &geometry, uint32_t gbpa) {
    char line[120];
    const unsigned row = kRowSid + (sid == kSidSec ? 2 : 0);
    snprintf(line, sizeof line, "SID %x %s | smr %08x #%u match %u t%u b%u", sid, sidVerdict(view, gbpa),
             view.found ? view.smr : 0, view.found ? view.index : 0, view.matches, view.type, view.bank);
    put(row, line);
    if (!view.found || !view.bankRead) {
        snprintf(line, sizeof line, "SID %x bank %u of %u, %s", sid, view.bank, geometry.banks,
                 view.found ? "past the geometry" : "no SMR");
        put(row + 1, line);
        return;
    }
    snprintf(line, sizeof line, "SID %x bank %u cbar %08x cba2r %08x sctlr %08x mask %x", sid, view.bank, view.cbar,
             view.cba2r, view.sctlr, view.mask);
    put(row + 1, line);
}
// The whole stream-match table, in four rows of up to 88 characters: "#i sid+mask>type/bank". It shows what the bootloader
// left behind, so a vendor tree that routes the display to another SID still shows up.
void rowsSmr(const SmmuGeometry &geometry) {
    char buffer[4][120] = {};
    unsigned row = 0, valid = 0;
    snprintf(buffer[0], sizeof buffer[0], "SMR %u groups: #i sid+mask>t/b", geometry.groups);
    for (unsigned i = 0; i < geometry.groups; ++i) {
        const uint32_t smr = smmuRead(kSmmuSmr + 4 * i);
        if (!(smr & 0x80000000u)) continue;
        const uint32_t s2cr = smmuRead(kSmmuS2cr + 4 * i);
        char entry[32];
        snprintf(entry, sizeof entry, " %u:%x+%x>%u/%u", i, smr & 0xffff, (smr >> 16) & 0x7fff, (s2cr >> 16) & 3,
                 s2cr & 0xff);
        valid = valid + 1;
        if (strlen(buffer[row]) + strlen(entry) > 88 && ++row >= 4) break;
        strlcat(buffer[row], entry, sizeof buffer[0]);
    }
    // No valid group at all: every stream (the display's included) passes untranslated unless sCR0.USFCFG faults it.
    if (!valid) strlcat(buffer[0], " none valid", sizeof buffer[0]);
    for (unsigned i = 0; i < 4; ++i) put(kRowSmr + i, buffer[i]);
}

// The compositor's surfaces as the display sees them: the descriptor it offers each frame and how many physically contiguous
// segments that memory has. A descriptor already seen costs one pointer compare; a new one walks its segments once. The count
// is U5 of the plan (how many surfaces the compositor really cycles through), the offers per second are the frame rate it
// presents at, which is one of the things the plan wanted from the INTF frame counter.
struct Surface {IOMemoryDescriptor *memory; uint32_t segments, frames;};
constexpr unsigned kSurfaces = 8;
Surface gSurfaces[kSurfaces];
volatile unsigned gSurfaceCount;
uint64_t gSurfaceFrames, gSurfaceSegments, gSurfaceSegmentMax;
uint64_t gLastOffers, gLastRoundUs;
unsigned gRate;

void surfaceOffered(IOMemoryDescriptor *memory) {
    if (!memory) return;
    gSurfaceFrames = gSurfaceFrames + 1;
    const unsigned count = gSurfaceCount;
    for (unsigned i = 0; i < count; ++i)
        if (gSurfaces[i].memory == memory) {gSurfaces[i].frames = gSurfaces[i].frames + 1; return;}
    if (count >= kSurfaces) return;
    Surface &surface = gSurfaces[count];
    surface.memory = memory;
    surface.frames = 1;
    surface.segments = 0;
    const IOByteCount length = (IOByteCount)memory->getLength();
    IOByteCount offset = 0;
    while (offset < length && surface.segments < 65535) {
        IOByteCount run = 0;
        const addr64_t physical = memory->getPhysicalSegment(offset, &run, kIOMemoryMapperNone);
        if (!physical || !run || run > length - offset) break;
        offset += run;
        surface.segments = surface.segments + 1;
    }
    gSurfaceCount = count + 1;
    gSurfaceSegments = gSurfaceSegments + surface.segments;
    if (surface.segments > gSurfaceSegmentMax) gSurfaceSegmentMax = surface.segments;
}
// The offers the driver was handed since the previous round and the rate they amount to. The first round runs 2 s after the
// late probe, before the compositor has presented anything, so a round's own numbers are what get reported: one line per round
// with the round's index makes that series readable (surf 0 in round 1 is "nothing yet", not "nothing ever").
unsigned gRound;
uint64_t gRoundOffers;
void measureRate() {
    const uint64_t now = uptimeUs(), offers = gSurfaceFrames;
    const bool measured = gLastRoundUs && now > gLastRoundUs && offers >= gLastOffers;
    gRoundOffers = measured ? offers - gLastOffers : 0;
    gRate = measured ? (unsigned)(gRoundOffers * 1000000ull / (now - gLastRoundUs)) : 0;
    gLastOffers = offers;
    gLastRoundUs = now;
    gRound = gRound + 1;
}

// One round. The decision row comes first and is computed before anything else is printed, so a log that is cut short still
// carries what P1 branches on.
void round() {
    char line[120];
    PE_Video video = {};
    const bool haveVideo = PE_current_console(&video) == 0;
    SmmuGeometry geometry = {};
    SidView unsec = {}, sec = {};
    uint32_t gbpa = 0;
    bool smmuUsable = false;
    if (gSmmuOk) {
        geometry = smmuGeometry();
        if (geometry.usable) {
            gbpa = smmuRead(kSmmuGbpa);
            unsec = analyseSid(kSidUnsec, geometry);
            sec = analyseSid(kSidSec, geometry);
            smmuUsable = true;
        }
    }
    measureRate();
    // The decision: which case the display stream is in, the frame buffer's own address and geometry, and what the plan's
    // U5 (surface count) and the presented rate came out as.
    const SidView *shown = unsec.found ? &unsec : sec.found ? &sec : nullptr;
    char variant[48];
    char letter = '?';
    if (!gSmmuOk) snprintf(variant, sizeof variant, "SMMU not mapped");
    else if (!smmuUsable) snprintf(variant, sizeof variant, "SMMU not answering");
    else if (!shown) snprintf(variant, sizeof variant, "no display SID in the table");
    else {
        const char *verdict = sidVerdict(*shown, gbpa);
        // The row already carries the case letter; the verdict's own first token would repeat it ("A bypass" -> "bypass").
        snprintf(variant, sizeof variant, "sid %x %s", shown == &sec ? kSidSec : kSidUnsec,
                 verdict[1] == ' ' ? verdict + 2 : verdict);
        letter = verdict[0] == 'A' || verdict[0] == 'B' || verdict[0] == 'C' ? verdict[0] : '!';
    }
    snprintf(line, sizeof line, "CASE %c: %s | fb %llx %llux%llu r%llu | surf %u fps %u", letter, variant,
             (unsigned long long)(haveVideo ? video.v_baseAddr : native_gop_physical),
             (unsigned long long)(haveVideo ? video.v_width : 0), (unsigned long long)(haveVideo ? video.v_height : 0),
             (unsigned long long)(haveVideo ? video.v_rowBytes : 0), gSurfaceCount, gRate);
    put(kRowCase, line);
    if (smmuUsable) {
        snprintf(line, sizeof line, "SMMU gbpa %08x idr %08x %08x %08x gfsr %08x", gbpa, smmuRead(kSmmuIdr0),
                 smmuRead(kSmmuIdr1), smmuRead(kSmmuIdr2), smmuRead(kSmmuGfsr));
        put(kRowSmmu, line);
        rowsSid(kSidUnsec, unsec, geometry, gbpa);
        rowsSid(kSidSec, sec, geometry, gbpa);
        rowsSmr(geometry);
    } else if (gSmmuOk) {
        snprintf(line, sizeof line, "SMMU unreadable: idr0 %08x idr1 %08x", smmuRead(kSmmuIdr0), smmuRead(kSmmuIdr1));
        put(kRowSmmu, line);
    } else {
        put(kRowSmmu, "SMMU not mapped");
    }
    // The frame buffer the firmware left: PE_Video in full, then the two mappings of it the driver and the presenter published.
    if (haveVideo) {
        snprintf(line, sizeof line, "PE_Video base %llx row %llu %llux%llu depth %llu",
                 (unsigned long long)video.v_baseAddr, (unsigned long long)video.v_rowBytes,
                 (unsigned long long)video.v_width, (unsigned long long)video.v_height, (unsigned long long)video.v_depth);
        put(kRowVideo, line);
        char format[65];
        for (unsigned i = 0; i < 64; ++i) format[i] = video.v_pixelFormat[i];
        format[64] = 0;
        snprintf(line, sizeof line, "PE_Video fmt %s disp %llu off %llu len %llu rot %u scale %u", format,
                 (unsigned long long)video.v_display, (unsigned long long)video.v_offset,
                 (unsigned long long)video.v_length, video.v_rotate, video.v_scale);
        put(kRowVideo2, line);
    } else {
        put(kRowVideo, "PE_Video: no console");
        put(kRowVideo2, "");
    }
    snprintf(line, sizeof line, "GOP pe %llx | presenter %llx | legacy %llx len %llu",
             (unsigned long long)(haveVideo ? video.v_baseAddr : 0), (unsigned long long)native_gop_physical,
             (unsigned long long)native_legacy_gop_physical, (unsigned long long)native_legacy_gop_bytes);
    put(kRowGop, line);
    snprintf(line, sizeof line, "surfaces %u (of %u) segs avg %llu max %llu | round %u: %llu offers %u/s", gSurfaceCount,
             kSurfaces, (unsigned long long)(gSurfaceCount ? gSurfaceSegments / gSurfaceCount : 0),
             (unsigned long long)gSurfaceSegmentMax, gRound, (unsigned long long)gRoundOffers, gRate);
    put(kRowSurfaces, line);
}

void mdssThread(void *, wait_result_t) {
    // Two seconds: early enough to sit in the log well before any reader timeout, late enough not to compete with the display
    // driver's start. Nothing here touches a display controller register, so no liveness proof is needed.
    IOSleep(2000);
    native_mdss_step = 1;
    gSmmuOk = mapWindow(kSmmuBase, kSmmuLength, gSmmuMap, gSmmu);
    // One register first: IDR0 has to answer before the rest of the block is touched.
    if (gSmmuOk) gSmmuOk = answers(smmuRead(kSmmuIdr0));
    put(kRowState, "state: MDSS not read (an SError from it panicked the kernel; see MdssProbe.cpp)");
    native_mdss_step = 2;
    for (;;) {
        round();
        native_mdss_step = 99;
        IOSleep(50000);
    }
}
}  // namespace

// Called from the late probe (LegacyDisplayProbe.cpp, after native_gpu_probe). No boot argument: there is nothing here that
// needs one, and nothing here is optional.
extern "C" void native_mdss_probe(void) {
    static bool started;
    if (started) return;
    started = true;
    thread_t thread = nullptr;
    if (kernel_thread_start(mdssThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
}

// Called by the display driver's offer path (LegacyDisplayProbe.cpp, native_legacy_gpu_offer) for every frame it hands on.
extern "C" void native_mdss_surface_offered(IOMemoryDescriptor *memory) {
    surfaceOffered(memory);
}

// The apps SMMU's window as this probe mapped it, or 0 before its first round has mapped it (FlipProbe.cpp's gate reuses it).
extern "C" uintptr_t native_mdss_smmu_window(void) {
    return gSmmu;
}
