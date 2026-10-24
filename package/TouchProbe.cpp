// SPDX-License-Identifier: BSD-2-Clause-Patent
// Default-off staged bring-up of pipa's touch controller (Novatek NT36532 on QUP wrapper 0 SE4 SPI), after UniXNU-sheng's
// TouchProbe.cpp (NT36532E on SM8550; its THP frames are replaced by the point reports pipa's firmware sends). `pipa-touch=<n>`:
//   1  read-only snapshot of the GCC branches, the S4 DFS levels and the touch pins
//   2  vote the QUP clocks, wait for them, configure the pins, read the GENI engine identity
//   3  GENI SPI init (FIFO mode, or SE DMA through an SMMU pass-through if the FIFO is disabled) and clock, an engine loopback test,
//      then reset the touch bootloader and read the chip id (drivers/NativeNovatekTouch nvt_identify)
//   4  host-download the firmware (the tablet's novatek_nt36532_m82_fw_tm.bin, built in by build-native-legacy-display.py), wait
//      for the reset state and read the firmware info (the coordinate range)
//   5  read point reports when the attention line (GPIO39, rising edge, latched by the TLMM without an interrupt) fires, and
//      publish the fingers to NativeHidInput (TouchState.cpp)
// Bits 8 and up of the value are a count of scripted strokes (autoTouchThread) for runs nobody watches: 10245 is stage 5 and 40.
// The rows below go to the kernel log every 10 s (BootRamoops.c); the first failure stops the sequence. Board constants and their
// sources are in TouchBoard.h, the controller protocol in NvtProtocol.c. Stages 1-5 run on the tablet since 2026-10-24.
#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <string.h>
#include <libkern/c++/OSSharedPtr.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
extern "C" {
#include "GeniSpi.h"
#include "NvtProtocol.h"
#include "TouchBoard.h"
#include "SmmuBypass.h"
extern const uint8_t native_touch_firmware[] __attribute__((weak_import));
extern const uint32_t native_touch_firmware_size __attribute__((weak_import));
// The finger set (TouchState.cpp) and its consumer's notification.
extern volatile uint32_t native_touch_seq, native_touch_count, native_touch_hid_reports;
extern volatile int native_touch_xs[12], native_touch_ys[12], native_touch_ids[12];
void native_touch_published(void);
}

extern "C" {
char native_touch_rows[4][120];
char native_touch_trace[8][120];
volatile uint32_t native_touch_loop_step;
volatile uint32_t native_touch_stage_req, native_touch_stage_done, native_touch_step, native_touch_frames,
    native_touch_valid, native_touch_contacts;
volatile int native_touch_error;
volatile int native_touch_last_x = -1, native_touch_last_y = -1;
// Finished strokes as published: a ring of {start ms, duration ms, first x, first y, last x, last y, reports} in HID units, and the
// extent every published contact reached {min x, max x, min y, max y}. For the log (BootRamoops.c) and the axis check.
volatile int32_t native_touch_strokes[16][7];
volatile uint32_t native_touch_stroke_count;
volatile int32_t native_touch_extent[4] = {0x7fffffff, -1, 0x7fffffff, -1};
}

namespace {
struct Region {OSSharedPtr<IOMemoryMap> keep; uintptr_t va = 0;};

bool mapRegion(uint64_t phys, uint64_t length, Region &region) {
    IODeviceMemory *memory = IODeviceMemory::withRange(phys, length);
    if (!memory) return false;
    region.keep = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
    memory->release();
    region.va = region.keep ? static_cast<uintptr_t>(region.keep->getVirtualAddress()) : 0;
    return region.va != 0;
}
uint32_t rd32(uintptr_t base, uint32_t reg) {return *reinterpret_cast<volatile uint32_t *>(base + reg);}
void wr32(uintptr_t base, uint32_t reg, uint32_t value) {
    *reinterpret_cast<volatile uint32_t *>(base + reg) = value;
    __asm__ volatile("dsb sy" ::: "memory");
}

// The serial engine's DMA master reads and writes two 64 KiB buffers (only when its FIFO is disabled). The apps SMMU is
// firmware-managed; the stream gets a pass-through mapping, so the engine addresses physical memory. Caches are maintained
// explicitly.
constexpr size_t kDmaBytes = 64 * 1024;
void cacheRange(void *p, size_t len, int op) {   // 0 clean, 1 clean+invalidate, 2 invalidate
    uint64_t ctr;
    __asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
    size_t line = 4u << ((ctr >> 16) & 15);
    uintptr_t a = reinterpret_cast<uintptr_t>(p) & ~(line - 1), end = reinterpret_cast<uintptr_t>(p) + len;
    for (; a < end; a += line) {
        if (op == 0) __asm__ volatile("dc cvac, %0" :: "r"(a) : "memory");
        else if (op == 1) __asm__ volatile("dc civac, %0" :: "r"(a) : "memory");
        else __asm__ volatile("dc ivac, %0" :: "r"(a) : "memory");
    }
    __asm__ volatile("dsb sy" ::: "memory");
}
void dmaClean(void *, void *p, size_t len) {cacheRange(p, len, 0);}
void dmaCleanInval(void *, void *p, size_t len) {cacheRange(p, len, 1);}
void dmaInval(void *, void *p, size_t len) {cacheRange(p, len, 2);}
uint32_t smmuRd(void *ctx, unsigned o) {return rd32(reinterpret_cast<uintptr_t>(ctx), o);}
void smmuWr(void *ctx, unsigned o, uint32_t v) {wr32(reinterpret_cast<uintptr_t>(ctx), o, v);}
int smmuErrorCode(const char *e) {
    static const char *const names[] = {"SMMU AArch64 stage 1 unsupported", "DMA exceeds SMMU address width", "unsupported SMMU geometry",
        "conflicting stream mapping", "reserved context bank is not a bypass bank", "no free stream-match group",
        "SMMU bypass bank readback", "SMMU stream context readback", "SMMU stream mapping readback"};
    for (unsigned i = 0; i < sizeof names / sizeof *names; ++i) if (strcmp(e, names[i]) == 0) return static_cast<int>(i) + 1;
    return 99;
}

uint32_t ioRd(void *ctx, uint32_t reg) {return rd32(reinterpret_cast<uintptr_t>(ctx), reg);}
void ioWr(void *ctx, uint32_t reg, uint32_t value) {wr32(reinterpret_cast<uintptr_t>(ctx), reg, value);}
void ioDelayUs(void *, unsigned us) {IODelay(us);}

struct Session {
    Region gccLow, gccQup, gccCore, gccVote, tlmmIrq, tlmmSpi, wrapper, se, smmu;
    IOBufferMemoryDescriptor *dmaMem;
    uint64_t dmaPhys;
    int smmuSlot, smmuBank, smmuErr;
    // First transfers as seen on the wire, and the identification result per SPI mode tried.
    unsigned traceN;
    uint32_t traceLen[8];
    uint8_t traceTx[8][8], traceRx[8][10];
    unsigned modeTried, modeUsed;
    // Last six transfers (ring) and the firmware being downloaded, for failures after identification.
    unsigned ringN;
    uint32_t ringLen[6];
    uint8_t ringTx[6][8], ringRx[6][8];
    unsigned fwParts, fwCasc, fwIlm, fwDlm, fwSize;
    // Report loop evidence: latched edges, reads, checksum failures, recovery patterns, reads without an edge, the last report.
    uint32_t edges, reads, bad, recov, idleReads, irqCfg;
    uint8_t last[24];
    int loopRc = 99;                  // engine loopback self-test (99 = not run)
    uint8_t loopRx[16];
    int modeRc[4];
    uint8_t modeId[4][6];
    struct geni_spi spi;
    uint8_t *tx = nullptr, *rx = nullptr;
    struct nvt_chip chip;
    struct nvt_bus bus;
    struct nvt_fw_info info;
    uint32_t vals[32];
    int lastRc = 0;
    uint32_t lastLen = 0, xIrq = 0, xGeni = 0, xTx = 0, xRx = 0;
    uint64_t levelHz[TB_DFS_LEVELS];
    unsigned level = 0, div = 0;
    uint64_t actualHz = 0;
};

int busXfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len) {
    Session *s = static_cast<Session *>(ctx);
    int rc = geni_spi_transfer(&s->spi, tx, rx, len, 0);
    {
        unsigned i = s->ringN++ % 6;
        s->ringLen[i] = static_cast<uint32_t>(len);
        for (unsigned k = 0; k < 8; ++k) {
            s->ringTx[i][k] = (tx && k < len) ? tx[k] : 0;
            s->ringRx[i][k] = (rx && k < len && !rc) ? rx[k] : 0;
        }
    }
    if (!rc && s->traceN < 8) {
        unsigned i = s->traceN++;
        s->traceLen[i] = static_cast<uint32_t>(len);
        for (unsigned k = 0; k < 8 && k < len && tx; ++k) s->traceTx[i][k] = tx[k];
        for (unsigned k = 0; k < 10 && k < len && rx; ++k) s->traceRx[i][k] = rx[k];
    }
    if (rc && !s->lastRc) {   // keep the engine state of the first failure
        s->lastRc = rc;
        s->lastLen = static_cast<uint32_t>(len);
        s->xIrq = s->spi.snap_irq;
        s->xGeni = s->spi.snap_status;
        s->xTx = s->spi.snap_txfs;
        s->xRx = s->spi.snap_rxfs;
    }
    return rc;
}
void busDelay(void *, unsigned ms) {IOSleep(ms);}

void fail(unsigned step, int code) {native_touch_step = step; native_touch_error = code;}
void rows(const Session &s);
void mark(const Session &s, unsigned step) {native_touch_step = step; rows(s);}

// snprintf returns the length it would have written, so a running offset must be clamped before the next append.
struct Out {
    char *p; size_t cap, n;
    void add(const char *fmt, unsigned a = 0, unsigned b = 0) {
        if (n + 1 >= cap) return;
        int w = snprintf(p + n, cap - n, fmt, a, b);
        if (w < 0) return;
        n = (n + static_cast<size_t>(w) >= cap) ? cap - 1 : n + static_cast<size_t>(w);
    }
    void hex(uint8_t v) {add("%02x", v);}
};

void traceRows(const Session &s) {
    for (unsigned r = 0; r < 2; ++r) {
        Out o = {native_touch_trace[r], sizeof native_touch_trace[r], 0};
        o.p[0] = 0;
        o.add("TCH xfer");
        for (unsigned j = 0; j < 2; ++j) {
            unsigned i = r * 2 + j;
            if (i >= s.traceN) break;
            o.add(" #%u len%u tx", i, s.traceLen[i]);
            for (unsigned k = 0; k < 8 && k < s.traceLen[i]; ++k) o.hex(s.traceTx[i][k]);
            o.add(" rx");
            for (unsigned k = 0; k < 10 && k < s.traceLen[i]; ++k) o.hex(s.traceRx[i][k]);
            o.add(" |");
        }
    }
    const struct nvt_chip &c = s.chip;
    Out v = {native_touch_trace[2], sizeof native_touch_trace[2], 0};
    v.p[0] = 0;
    v.add("TCH crc flags%02x", c.crc_flags);
    v.add(" ilm %08x/", c.crc_golden[0]);
    v.add("%08x dlm ", c.crc_result[0]);
    v.add("%08x/", c.crc_golden[1]);
    v.add("%08x sram ilm bad%u", c.crc_result[1], c.sram_bad[0]);
    v.add(" dlm bad%u", c.sram_bad[1]);
    Out f = {native_touch_trace[3], sizeof native_touch_trace[3], 0};
    f.p[0] = 0;
    f.add("TCH reports lp%u", native_touch_loop_step);
    f.add(" edges%u", s.edges);
    f.add(" reads%u", s.reads);
    f.add(" bad%u", s.bad);
    f.add(" recov%u", s.recov);
    f.add(" idle%u cfg%08x last", s.idleReads, s.irqCfg);
    for (unsigned k = 0; k < 24; ++k) f.hex(s.last[k]);
    Out d = {native_touch_trace[4], sizeof native_touch_trace[4], 0};
    d.p[0] = 0;
    d.add("TCH dl try%u", c.dl_attempt);
    d.add(" stage%u", c.dl_stage);
    d.add(" rc%d", static_cast<unsigned>(c.dl_rc));
    d.add(" wait%02x", c.dl_wait_value);
    d.add(" polls%u rst", c.dl_polls);
    for (unsigned k = 0; k < 5; ++k) d.hex(c.dl_reset_state[k]);
    d.add(" fw parts%u", s.fwParts);
    d.add(" casc%u", s.fwCasc);
    d.add(" ilm%u", s.fwIlm);
    d.add(" dlm%u", s.fwDlm);
    for (unsigned r = 0; r < 2; ++r) {
        Out o = {native_touch_trace[5 + r], sizeof native_touch_trace[5 + r], 0};
        o.p[0] = 0;
        o.add("TCH last");
        for (unsigned j = 0; j < 2; ++j) {
            unsigned k = r * 2 + j;                       // last four, oldest first
            if (s.ringN < 4 && k >= s.ringN) break;
            unsigned seq = s.ringN >= 4 ? s.ringN - 4 + k : k;
            unsigned i = seq % 6;
            o.add(" #%u len%u tx", seq, s.ringLen[i]);
            for (unsigned q = 0; q < 8 && q < s.ringLen[i]; ++q) o.hex(s.ringTx[i][q]);
            o.add(" rx");
            for (unsigned q = 0; q < 8 && q < s.ringLen[i]; ++q) o.hex(s.ringRx[i][q]);
            o.add(" |");
        }
    }
    Out m = {native_touch_trace[7], sizeof native_touch_trace[7], 0};
    m.p[0] = 0;
    m.add("TCH modes tried%u used%u", s.modeTried, s.modeUsed);
    for (unsigned i = 0; i < s.modeTried && i < 4; ++i) {
        m.add(" | rc%d id", static_cast<unsigned>(s.modeRc[i]));
        for (unsigned k = 0; k < 6; ++k) m.hex(s.modeId[i][k]);
    }
    m.add(" | loop rc%d rx", static_cast<unsigned>(s.loopRc));
    for (unsigned k = 0; k < 8; ++k) m.hex(s.loopRx[k]);
}

void rows(const Session &s) {
    traceRows(s);
    const uint32_t *v = s.vals;
    snprintf(native_touch_rows[0], sizeof native_touch_rows[0],
        "TCH s%u/%u st%u e%d vote%08x h%08x %08x %08x %08x %08x gp%08x dfs%08x",
        native_touch_stage_done, native_touch_stage_req, native_touch_step, native_touch_error,
        v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
    snprintf(native_touch_rows[1], sizeof native_touch_rows[1],
        "TCH p%08x %08x %08x %08x irq%08x wr%08x fw%08x if%08x hw%08x L%u/%u %uHz",
        v[9], v[10], v[11], v[12], v[13], v[14], v[15], v[16], v[17], s.level, s.div, (unsigned)s.actualHz);
    snprintf(native_touch_rows[2], sizeof native_touch_rows[2],
        "TCH id%02x%02x%02x%02x%02x%02x %.15s v%02x p%04x %ux%u max%ux%u rp%u ok%u c%u %d,%d hid%u",
        s.chip.id[0], s.chip.id[1], s.chip.id[2], s.chip.id[3], s.chip.id[4], s.chip.id[5],
        s.chip.map ? s.chip.map->name : "-", s.info.fw_ver, s.info.pid, s.info.x_num, s.info.y_num, s.info.abs_x_max,
        s.info.abs_y_max, native_touch_frames, native_touch_valid, native_touch_contacts, native_touch_last_x, native_touch_last_y,
        native_touch_hid_reports);
    snprintf(native_touch_rows[3], sizeof native_touch_rows[3],
        "TCH xf rc%d n%u ev%08x gs%08x tx%08x rx%08x dma t%x r%x ti%x ri%x sm%d/%d/e%d pa%llx",
        s.lastRc, s.lastLen, s.xIrq, s.xGeni, s.xTx, s.xRx, s.spi.snap_dma_tx, s.spi.snap_dma_rx, s.spi.snap_dma_txin,
        s.spi.snap_dma_rxin, s.smmuSlot, s.smmuBank, s.smmuErr, (unsigned long long)s.dmaPhys);
}

struct Stroke {int id, x0, y0, x1, y1, frames; uint32_t t0;};
Stroke strokes[NVT_POINT_SLOTS];
unsigned strokeCount;
uint32_t touchMs() {
    uint64_t f = 0; __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(f));
    return (uint32_t)(mach_absolute_time() / ((f ? f : 19200000) / 1000));
}
void trackStrokes(unsigned count, const int *ids, const int *xs, const int *ys) {
    const uint32_t now = touchMs();
    for (unsigned i = 0; i < count; ++i) {
        if (xs[i] < native_touch_extent[0]) native_touch_extent[0] = xs[i];
        if (xs[i] > native_touch_extent[1]) native_touch_extent[1] = xs[i];
        if (ys[i] < native_touch_extent[2]) native_touch_extent[2] = ys[i];
        if (ys[i] > native_touch_extent[3]) native_touch_extent[3] = ys[i];
        unsigned k = 0;
        while (k < strokeCount && strokes[k].id != ids[i]) ++k;
        if (k == strokeCount) {
            if (strokeCount == NVT_POINT_SLOTS) continue;
            strokes[strokeCount++] = {ids[i], xs[i], ys[i], xs[i], ys[i], 0, now};
        }
        strokes[k].x1 = xs[i]; strokes[k].y1 = ys[i]; ++strokes[k].frames;
    }
    for (unsigned k = 0; k < strokeCount;) {
        bool live = false;
        for (unsigned i = 0; i < count && !live; ++i) live = ids[i] == strokes[k].id;
        if (live) {++k; continue;}
        volatile int32_t *r = native_touch_strokes[native_touch_stroke_count & 15];
        r[0] = (int32_t)strokes[k].t0; r[1] = (int32_t)(now - strokes[k].t0);
        r[2] = strokes[k].x0; r[3] = strokes[k].y0; r[4] = strokes[k].x1; r[5] = strokes[k].y1; r[6] = strokes[k].frames;
        native_touch_stroke_count = native_touch_stroke_count + 1;
        strokes[k] = strokes[--strokeCount];
    }
}

// The finger set in the digitizer's units (x 0..30480 across the firmware's x range, y 0..20320 across its y range), under the
// sequence lock TouchState.cpp describes, then the HID input's notification. Whether the firmware's axes match the panel's
// (orientation, mirroring) is for the first strokes on the tablet to show (native_touch_strokes).
void publish(unsigned count, const nvt_point *p, const nvt_fw_info &info) {
    int ids[NVT_POINT_SLOTS], xs[NVT_POINT_SLOTS], ys[NVT_POINT_SLOTS];
    const unsigned xmax = info.abs_x_max ? info.abs_x_max : 1, ymax = info.abs_y_max ? info.abs_y_max : 1;
    for (unsigned i = 0; i < count && i < NVT_POINT_SLOTS; ++i) {
        ids[i] = p[i].id;
        xs[i] = static_cast<int>(p[i].x * 30480u / xmax);
        ys[i] = static_cast<int>(p[i].y * 20320u / ymax);
    }
    if (count > NVT_POINT_SLOTS) count = NVT_POINT_SLOTS;
    trackStrokes(count, ids, xs, ys);
    native_touch_seq = native_touch_seq + 1;
    __asm__ volatile("dmb ish" ::: "memory");
    native_touch_count = count;
    for (unsigned i = 0; i < count; ++i) {native_touch_xs[i] = xs[i]; native_touch_ys[i] = ys[i]; native_touch_ids[i] = ids[i];}
    __asm__ volatile("dmb ish" ::: "memory");
    native_touch_seq = native_touch_seq + 1;
    native_touch_published();
}

bool snapshot(Session &s) {
    mark(s, 1);
    if (!mapRegion(TB_GCC_BASE + 0x0000, 0x1000, s.gccLow) || !mapRegion(TB_GCC_BASE + 0x17000, 0x1000, s.gccQup) ||
        !mapRegion(TB_GCC_BASE + 0x23000, 0x1000, s.gccCore) || !mapRegion(TB_GCC_BASE + 0x52000, 0x1000, s.gccVote) ||
        !mapRegion(TB_TLMM_BASE + tb_tlmm_ctl(TB_GPIO_IRQ), 0x1000, s.tlmmIrq) ||
        !mapRegion(TB_TLMM_BASE + tb_tlmm_ctl(TB_GPIO_SPI_FIRST), 4 * TB_GPIO_STRIDE, s.tlmmSpi)) {
        fail(1, -12);
        return false;
    }
    uint32_t *v = s.vals;
    v[0] = rd32(s.gccVote.va, TB_GCC_VOTE_ENABLE & 0xfff);
    v[1] = rd32(s.gccQup.va, TB_HALT_M_AHB & 0xfff);
    v[2] = rd32(s.gccQup.va, TB_HALT_S_AHB & 0xfff);
    v[3] = rd32(s.gccCore.va, TB_HALT_CORE & 0xfff);
    v[4] = rd32(s.gccCore.va, TB_HALT_CORE_2X & 0xfff);
    v[5] = rd32(s.gccQup.va, TB_HALT_S4 & 0xfff);
    v[6] = rd32(s.gccLow.va, TB_GCC_GPLL0_MODE);
    v[7] = rd32(s.gccQup.va, (TB_S4_RCG + TB_RCG_DFSR_CMD) & 0xfff);
    v[8] = rd32(s.gccQup.va, (TB_S4_RCG + TB_RCG_CFG) & 0xfff);
    // With DFS on (DFSR bit 0) the levels are the PERF registers; otherwise the plain RCG's one rate stands for every level.
    const bool dfs = (v[7] & 1) != 0;
    for (unsigned l = 0; l < TB_DFS_LEVELS; ++l) {
        uint32_t cfg = dfs ? rd32(s.gccQup.va, (TB_S4_RCG + TB_RCG_PERF_CFG(l)) & 0xfff) : v[8];
        uint32_t m = dfs ? rd32(s.gccQup.va, (TB_S4_RCG + TB_RCG_PERF_M(l)) & 0xfff) : rd32(s.gccQup.va, (TB_S4_RCG + 0x8) & 0xfff);
        uint32_t n = dfs ? rd32(s.gccQup.va, (TB_S4_RCG + TB_RCG_PERF_N(l)) & 0xfff) : rd32(s.gccQup.va, (TB_S4_RCG + 0xc) & 0xfff);
        s.levelHz[l] = tb_dfs_level_hz(cfg, m, n);
        if (l < 4) s.vals[24 + l] = cfg;
        if (!dfs) break;
    }
    for (unsigned i = 0; i < 4; ++i) v[9 + i] = rd32(s.tlmmSpi.va, i * TB_GPIO_STRIDE);
    v[13] = rd32(s.tlmmIrq.va, 4);
    return true;
}

bool enable(Session &s) {
    mark(s, 2);
    // GPLL0 feeds the DFS levels; vote for it (a shared vote, not an exclusive enable) and wait for lock.
    wr32(s.gccVote.va, TB_GCC_GPLL0_ENABLE & 0xfff, rd32(s.gccVote.va, TB_GCC_GPLL0_ENABLE & 0xfff) | 1u);
    for (unsigned i = 0; i < 10000 && !(rd32(s.gccLow.va, TB_GCC_GPLL0_MODE) & (1u << 31)); ++i) IODelay(10);
    s.vals[6] = rd32(s.gccLow.va, TB_GCC_GPLL0_MODE);
    wr32(s.gccVote.va, TB_GCC_VOTE_ENABLE & 0xfff, rd32(s.gccVote.va, TB_GCC_VOTE_ENABLE & 0xfff) | TB_VOTE_ALL);
    // Branches with hardware clock gating (bit 1) show CLK_OFF while idle and wake on access, so only a branch without that bit
    // must have CLK_OFF clear.
    auto running = [](uint32_t cbcr) {return !(cbcr & TB_CBCR_CLK_OFF) || (cbcr & 2u);};
    bool ok = false;
    for (unsigned i = 0; i < 2000 && !ok; ++i) {
        ok = running(rd32(s.gccQup.va, TB_HALT_M_AHB & 0xfff)) && running(rd32(s.gccQup.va, TB_HALT_S_AHB & 0xfff)) &&
             !(rd32(s.gccCore.va, TB_HALT_CORE & 0xfff) & TB_CBCR_CLK_OFF) &&
             !(rd32(s.gccCore.va, TB_HALT_CORE_2X & 0xfff) & TB_CBCR_CLK_OFF) &&
             !(rd32(s.gccQup.va, TB_HALT_S4 & 0xfff) & TB_CBCR_CLK_OFF);
        if (!ok) IODelay(10);
    }
    s.vals[0] = rd32(s.gccVote.va, TB_GCC_VOTE_ENABLE & 0xfff);
    s.vals[1] = rd32(s.gccQup.va, TB_HALT_M_AHB & 0xfff);
    s.vals[2] = rd32(s.gccQup.va, TB_HALT_S_AHB & 0xfff);
    s.vals[3] = rd32(s.gccCore.va, TB_HALT_CORE & 0xfff);
    s.vals[4] = rd32(s.gccCore.va, TB_HALT_CORE_2X & 0xfff);
    s.vals[5] = rd32(s.gccQup.va, TB_HALT_S4 & 0xfff);
    if (!ok) {fail(2, -110); return false;}   // never touch the engine without running clocks
    const uint32_t cfg = tb_pin_cfg(TB_FUNC_QUP4, TB_DRIVE_6MA);
    for (unsigned i = 0; i < 4; ++i) wr32(s.tlmmSpi.va, i * TB_GPIO_STRIDE, cfg);
    for (unsigned i = 0; i < 4; ++i) s.vals[9 + i] = rd32(s.tlmmSpi.va, i * TB_GPIO_STRIDE);
    mark(s, 3);   // the next accesses touch the engine: if the log stops here, the engine access hung
    if (!mapRegion(TB_QUP_WRAPPER_BASE, 0x2000, s.wrapper) || !mapRegion(TB_SE_BASE, TB_SE_SIZE, s.se)) {fail(3, -12); return false;}
    s.spi.se = (struct geni_io){reinterpret_cast<void *>(s.se.va), ioRd, ioWr, ioDelayUs};
    s.spi.wrapper = (struct geni_io){reinterpret_cast<void *>(s.wrapper.va), ioRd, ioWr, ioDelayUs};
    mark(s, 31);   // first engine register read
    int r = geni_spi_read_info(&s.spi);
    s.vals[14] = s.spi.info.qup_hw_version;
    s.vals[15] = s.spi.info.fw_revision;
    s.vals[16] = s.spi.info.if_disable;
    s.vals[17] = s.spi.info.hw_param0;
    if (r) {fail(3, r); return false;}
    return true;
}

bool prepareDma(Session &s) {
    mark(s, 41);
    s.dmaMem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, static_cast<IOOptionBits>(kIODirectionInOut) | static_cast<IOOptionBits>(kIOMemoryPhysicallyContiguous),
                                                               2 * kDmaBytes, 0xfffffff000ULL);
    if (!s.dmaMem || s.dmaMem->prepare() != kIOReturnSuccess) {fail(41, -12); return false;}
    mark(s, 42);
    addr64_t seg_len = 0;
    s.dmaPhys = s.dmaMem->getPhysicalSegment(0, &seg_len);
    uint8_t *virt = static_cast<uint8_t *>(s.dmaMem->getBytesNoCopy());
    if (!virt || !s.dmaPhys || seg_len < 2 * kDmaBytes) {fail(42, -14); return false;}
    memset(virt, 0, 2 * kDmaBytes);
    cacheRange(virt, 2 * kDmaBytes, 1);
    if (!mapRegion(TB_APPS_SMMU_BASE, TB_APPS_SMMU_SIZE, s.smmu)) {fail(43, -12); return false;}
    mark(s, 43);
    struct smmu_io io = {reinterpret_cast<void *>(s.smmu.va), smmuRd, smmuWr};
    struct smmu_plan plan = smmu_plan_bypass(&io, TB_QUP_SMMU_SID, s.dmaPhys, 2 * kDmaBytes);
    s.smmuSlot = static_cast<int>(plan.slot); s.smmuBank = static_cast<int>(plan.bank);
    if (plan.error) {s.smmuErr = smmuErrorCode(plan.error); fail(43, -100 - s.smmuErr); return false;}
    const char *e = smmu_configure_bypass(&io, &plan, TB_QUP_SMMU_SID);
    if (e) {s.smmuErr = smmuErrorCode(e); fail(44, -100 - s.smmuErr); return false;}
    s.spi.use_dma = 1;
    s.spi.dma.ctx = nullptr;
    s.spi.dma.clean = dmaClean;
    s.spi.dma.clean_inval = dmaCleanInval;
    s.spi.dma.inval = dmaInval;
    s.spi.dma.tx_virt = virt;
    s.spi.dma.rx_virt = virt + kDmaBytes;
    s.spi.dma.tx_phys = s.dmaPhys;
    s.spi.dma.rx_phys = s.dmaPhys + kDmaBytes;
    s.spi.dma.size = kDmaBytes;
    return true;
}

bool openBus(Session &s) {
    mark(s, 4);
    s.spi.poll_limit = 400000;
    s.spi.poll_only = 2;             // keep the top-level IRQ enable: the status registers must stay visible to the poll
    // sheng's engine had FIFO_IF_DISABLE set by its firmware (GPI DMA mode) and needed the SE DMA path; pipa's is read in stage 2.
    if (s.spi.info.fifo_disabled && !prepareDma(s)) return false;
    int r = geni_spi_init(&s.spi);
    if (r) {fail(4, r); return false;}
    return true;
}

bool configureClock(Session &s, unsigned hz) {
    struct tb_clock_choice choice = {};
    if (tb_choose_clock(s.levelHz, TB_DFS_LEVELS, s.spi.oversampling, hz, &choice)) {fail(5, -22); return false;}
    s.level = choice.level;
    s.div = choice.div;
    s.actualHz = choice.actual_hz;
    int r = geni_spi_set_clock(&s.spi, hz, static_cast<unsigned>(choice.sclk_hz), choice.level);
    if (r) {fail(5, r); return false;}
    r = geni_spi_setup(&s.spi, TB_SPI_MODE, 0);
    if (r) {fail(5, r); return false;}
    return true;
}

// The attention line: GPIO39's TLMM interrupt detector set to rising edge (the DT's 0x2001) with its raw status latched but no
// interrupt delivered (enable bit 0 clear), so the loop polls one register instead of taking an interrupt. pinctrl-sm8250.c
// intr_cfg fields: enable 0, polarity 1, detection 3:2 (1 = rising edge), raw status 4, target 7:5; intr_status bit 0.
constexpr uint32_t kIntrCfg = 0x8, kIntrStatus = 0xc;
void armAttention(Session &s) {
    wr32(s.tlmmIrq.va, kIntrCfg, (1u << 4) | (1u << 2) | (1u << 1));
    wr32(s.tlmmIrq.va, kIntrStatus, 0);
    s.irqCfg = rd32(s.tlmmIrq.va, kIntrCfg);
}
bool attention(Session &s) {
    if (!(rd32(s.tlmmIrq.va, kIntrStatus) & 1)) return false;
    wr32(s.tlmmIrq.va, kIntrStatus, 0);
    return true;
}
}  // namespace

// The firmware's coordinate range once the report loop runs, for the scripted strokes (autoTouchThread).
static nvt_fw_info gAutoInfo;
static volatile bool gAutoReady;

static void touchThread(void *, wait_result_t) {
    Session *s = static_cast<Session *>(IOMallocType(Session));
    if (!s) {fail(0, -12); return;}
    bzero(s, sizeof *s);
    s->loopRc = 99;
    unsigned stage = native_touch_stage_req, hz = TB_SPI_DEFAULT_HZ;
    PE_parse_boot_argn("pipa-touch-hz", &hz, sizeof hz);
    if (hz < 100000 || hz > TB_SPI_MAX_HZ) hz = TB_SPI_DEFAULT_HZ;
    do {
        if (!snapshot(*s)) break;
        native_touch_stage_done = 1;
        rows(*s);
        if (stage < 2) break;
        if (!enable(*s)) break;
        native_touch_stage_done = 2;
        rows(*s);
        if (stage < 3) break;
        if (!openBus(*s) || !configureClock(*s, hz)) break;
        s->tx = static_cast<uint8_t *>(IOMalloc(NVT_TRANSFER_LEN + 8));
        s->rx = static_cast<uint8_t *>(IOMalloc(NVT_READ_MAX + 2));
        if (!s->tx || !s->rx) {fail(6, -12); break;}
        s->bus = (struct nvt_bus){s, busXfer, busDelay};
        mark(*s, 7);
        // Engine loopback: MOSI is fed back to MISO inside the controller, isolating the FIFO/DMA, packing and clock path from the
        // touch chip.
        {
            static const uint8_t pattern[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00};
            geni_spi_setup(&s->spi, GENI_SPI_LOOP, 0);
            s->loopRc = geni_spi_transfer(&s->spi, pattern, s->rx, sizeof pattern, 0);
            for (unsigned k = 0; k < 16; ++k) s->loopRx[k] = s->rx[k];
            geni_spi_setup(&s->spi, TB_SPI_MODE, 0);
            rows(*s);
        }
        // SPI mode: pipa-touch-mode=0..3 forces one; otherwise mode 0 (the vendor's) first, then the others until the chip id
        // matches, as a hardware cross-check.
        unsigned forced = 4;
        PE_parse_boot_argn("pipa-touch-mode", &forced, sizeof forced);
        static const unsigned order[4] = {0, 3, 1, 2};
        int r = NVT_ERR_CHIP;
        for (unsigned i = 0; i < 4 && r; ++i) {
            unsigned m = forced < 4 ? forced : order[i];
            geni_spi_setup(&s->spi, ((m & 1) ? GENI_SPI_CPHA : 0) | ((m & 2) ? GENI_SPI_CPOL : 0), 0);
            s->chip = {};
            s->chip.bus = &s->bus;
            s->chip.scratch = s->tx;
            s->chip.rxbuf = s->rx;
            r = nvt_identify(&s->chip);
            for (unsigned k = 0; k < 6; ++k) s->modeId[i][k] = s->chip.id[k];
            s->modeRc[i] = r;
            s->modeTried = i + 1;
            s->modeUsed = m;
            rows(*s);
            if (forced < 4) break;
        }
        if (r) {fail(7, r); rows(*s); break;}
        native_touch_stage_done = 3;
        rows(*s);
        if (stage < 4) break;
        if (!&native_touch_firmware || !&native_touch_firmware_size || !native_touch_firmware_size) {fail(8, -2); break;}
        mark(*s, 8);
        struct nvt_fw fw;
        r = nvt_fw_parse(&fw, native_touch_firmware, native_touch_firmware_size);
        if (r) {fail(8, r); break;}
        s->fwParts = fw.count; s->fwCasc = static_cast<unsigned>(fw.cascade_2nd_header);
        s->fwIlm = fw.part[0].size; s->fwDlm = fw.part[1].size; s->fwSize = static_cast<unsigned>(native_touch_firmware_size);
        mark(*s, 9);
        s->chip.reset_retries = 60;   // vendor: 10 polls of 10 ms; longer here to tell "slow" from "never"
        r = nvt_download_firmware(&s->chip, &fw);
        if (r) {fail(9, r); rows(*s); break;}
        mark(*s, 10);
        r = nvt_start_reports(&s->chip, &s->info);
        if (r == NVT_ERR_BUS || (r && !s->info.abs_x_max)) {fail(10, r); rows(*s); break;}
        native_touch_stage_done = 4;
        rows(*s);
        if (stage < 5) break;
        armAttention(*s);
        mark(*s, 11);
        gAutoInfo = s->info;
        gAutoReady = true;
        uint8_t data[NVT_POINT_DATA_LEN];
        unsigned sinceRead = 0, pass = 0, published = 0;
        unsigned wdt = 0;
        for (;;) {
            // The rows about every half second, whatever the reports do (the log copies them every 10 s).
            if (++pass % 250 == 0) rows(*s);
            native_touch_loop_step = 1;
            // A report when the line rose; without an edge for 500 ms, one anyway, in case the latch missed one. On the tablet
            // (2026-10-24) every finger report came with an edge, the lift included (676 edges, 671 reports in 5 strokes), and the
            // reads without one mostly returned 0xff (a bad checksum); 50 ms read the chip 6385 times in 6 minutes for nothing.
            if (!attention(*s)) {
                if (++sinceRead < 250) {IOSleep(2); continue;}
                ++s->idleReads;
            } else {
                ++s->edges;
            }
            sinceRead = 0;
            native_touch_loop_step = 2;
            if (nvt_read_points(&s->chip, data)) {fail(11, NVT_ERR_BUS); IOSleep(8); continue;}
            ++s->reads;
            for (unsigned q = 0; q < 24; ++q) s->last[q] = data[q];
            native_touch_loop_step = 3;
            int recovery = nvt_points_recovery(data);
            if (recovery) {
                ++s->recov;
                // The stock driver downloads the firmware again after ten watchdog patterns in a row.
                if (recovery == 2 && ++wdt >= 10) {
                    wdt = 0;
                    native_touch_loop_step = 4;
                    if (!nvt_download_firmware(&s->chip, &fw)) {(void)nvt_start_reports(&s->chip, &s->info); armAttention(*s);}
                }
                continue;
            }
            wdt = 0;
            if (!nvt_points_checksum_ok(data)) {++s->bad; continue;}
            native_touch_valid = native_touch_valid + 1;
            struct nvt_point pts[NVT_POINT_SLOTS];
            unsigned n = nvt_parse_points(data, &s->info, pts, NVT_POINT_SLOTS);
            native_touch_frames = native_touch_frames + 1;
            native_touch_contacts = n;
            if (n) {native_touch_last_x = pts[0].x; native_touch_last_y = pts[0].y;}
            native_touch_loop_step = 5;
            // An empty report only once after the last finger lifted: the reads without an edge return empty reports too.
            if (n || published) publish(n, pts, s->info);
            published = n;
        }
    } while (0);
    rows(*s);
}

// Scripted strokes for runs nobody watches (pipa-touch bits 8 and up: their count), as UniXNU-tb128fu's. Once the GPU hand-off has
// completed frames (the lock screen is up) and 30 s more have passed, a swipe up from the bottom edge unlocks to the home screen
// (the user's own unlock swipes of 2026-10-24 ran from HID y 20200 of 20320 to about 15500 in 250 to 360 ms), and 10 s later
// strokes alternately right to left and back across the middle of the panel turn its pages, each 300 ms long with 1.5 s between
// them: one finger in slot 9, which the controller's reports of one finger do not use, in the firmware's units.
extern "C" volatile uint64_t native_gop_gpu_completions __attribute__((weak_import));
extern "C" volatile uint32_t native_touch_auto_done;
volatile uint32_t native_touch_auto_done;
static void autoTouchThread(void *arg, wait_result_t) {
    const unsigned strokes = static_cast<unsigned>(reinterpret_cast<uintptr_t>(arg));
    while (!gAutoReady || !&native_gop_gpu_completions || !native_gop_gpu_completions) IOSleep(1000);
    IOSleep(30000);
    const int w = gAutoInfo.abs_x_max, h = gAutoInfo.abs_y_max;
    // From the panel's last rows, where NativeHidInput marks the contact as an edge one (SpringBoard's home gesture needs it).
    for (int step = 0; step <= 25; ++step) {
        const nvt_point p = {9, 1, static_cast<uint16_t>(w / 2), static_cast<uint16_t>(h - h / 300 - (h * 55 / 100) * step / 25), 8, 50};
        publish(1, &p, gAutoInfo);
        IOSleep(10);
    }
    publish(0, nullptr, gAutoInfo);
    // With 5 s after the unlock some rounds' strokes alternated between the first page and the Today view instead of the first and
    // second pages (frame buffer pictures, 2026-10-24), presumably because the first page turn (right to left) came too early: 10 s.
    IOSleep(10000);
    for (unsigned i = 0; i < strokes; ++i) {
        const int from = (i & 1) ? w / 5 : w * 4 / 5, to = (i & 1) ? w * 4 / 5 : w / 5;
        for (int step = 0; step <= 30; ++step) {
            const nvt_point p = {9, 1, static_cast<uint16_t>(from + (to - from) * step / 30), static_cast<uint16_t>(h / 2), 8, 50};
            publish(1, &p, gAutoInfo);
            IOSleep(10);
        }
        publish(0, nullptr, gAutoInfo);
        native_touch_auto_done = i + 1;
        IOSleep(1500);
    }
}

extern "C" void native_touch_probe(void) {
    unsigned value = 0;
    if (!PE_parse_boot_argn("pipa-touch", &value, sizeof value) || (value & 0xff) < 1 || native_touch_stage_req) return;
    unsigned stage = value & 0xff;
    if (stage > 5) stage = 5;
    native_touch_stage_req = stage;
    thread_t thread = nullptr;
    if (kernel_thread_start(touchThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    else native_touch_error = -12;
    if (stage == 5 && (value >> 8)) {
        thread_t automatic = nullptr;
        if (kernel_thread_start(autoTouchThread, reinterpret_cast<void *>(static_cast<uintptr_t>(value >> 8)), &automatic) == KERN_SUCCESS)
            thread_deallocate(automatic);
    }
}
