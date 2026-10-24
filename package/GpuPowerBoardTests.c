// Host test for pipa's GpuPowerBoard.h: the gated stages on a register model that reacts as the hardware is described to (a
// GDSC that powers up once SW_COLLAPSE clears, voted branches whose CLK_OFF follows the enable bit, an RCG whose root turns on and
// whose update completes), and the command-DB lookup on a synthetic database.
// cc -std=c11 -Wall -Wextra -Werror package/GpuPowerBoardTests.c && ./a.out
#include "GpuPowerBoard.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(c) do{if(!(c)){printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c);exit(1);}}while(0)

static uint32_t win[GPB_WINDOWS][0x9000 / 4];
static unsigned writes, cx_writes;
static int rcg_stuck;                    // the GMU RCG's root never turns on
static int rd(void *c, unsigned w, uint32_t off, uint32_t *out) {(void)c; if (w >= GPB_WINDOWS || off >= 0x9000) return 1; *out = win[w][off / 4]; return 0;}
static void branch(unsigned w, uint32_t off, uint32_t v) {win[w][off / 4] = (v & GPB_CBCR_ENABLE) ? (v & ~GPB_CBCR_CLK_OFF) : (v | GPB_CBCR_CLK_OFF);}
static int wr(void *c, unsigned w, uint32_t off, uint32_t v) {
    (void)c;
    if (w >= GPB_WINDOWS || off >= 0x9000) return 1;
    ++writes;
    if (w == GPB_WIN_GPUCC && off == GPB_CC_CX_GDSCR) {
        ++cx_writes;
        win[w][off / 4] = v;
        if (!(v & GPB_GDSC_SW_COLLAPSE)) {win[w][off / 4] |= GPB_GDSC_PWR_ON; win[w][GPB_CC_CX_GDS_HW_CTRL / 4] |= GPB_GDSC_PWR_ON;}
        return 0;
    }
    if (w == GPB_WIN_GPUCC && off == GPB_CC_GMU_RCGR) {
        uint32_t cmd = v & ~GPB_RCG_UPDATE;                     // the update completes at once
        cmd = (v & GPB_RCG_ROOT_EN) && !rcg_stuck ? (cmd & ~GPB_RCG_ROOT_OFF) : (cmd | GPB_RCG_ROOT_OFF);
        win[w][off / 4] = cmd;
        return 0;
    }
    if ((w == GPB_WIN_GPUCC && (off == GPB_CC_AHB_CLK || off == GPB_CC_SMMU_VOTE_CLK || off == GPB_CC_CX_GMU_CLK || off == GPB_CC_CXO_CLK)) ||
        (w == GPB_WIN_GCC_GPU && (off == GPB_GCC_DDRSS_GPU_AXI || off == GPB_GCC_GPU_MEMNOC_GFX))) {branch(w, off, v); return 0;}
    win[w][off / 4] = v;
    return 0;
}
static void reset_model(void) {
    memset(win, 0, sizeof win); writes = cx_writes = 0; rcg_stuck = 0;
    win[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 0x80000000;      // off
    win[GPB_WIN_GPUCC][GPB_CC_CX_GDSCR / 4] = 0x00222001;           // collapsed
    win[GPB_WIN_GPUCC][GPB_CC_GX_GDSCR / 4] = 0x00222001;
    win[GPB_WIN_GPUCC][GPB_CC_AHB_CLK / 4] = 0x80000000;
    win[GPB_WIN_GPUCC][GPB_CC_SMMU_VOTE_CLK / 4] = 0x80000000;
    win[GPB_WIN_GPUCC][GPB_CC_CX_GMU_CLK / 4] = 0x80000000;
    win[GPB_WIN_GPUCC][GPB_CC_CXO_CLK / 4] = 0x80000000;
    win[GPB_WIN_GPUCC][GPB_CC_GMU_RCGR / 4] = GPB_RCG_ROOT_OFF;
    win[GPB_WIN_GCC_GPU][GPB_GCC_DDRSS_GPU_AXI / 4] = 0x80000000;
    win[GPB_WIN_GCC_GPU][GPB_GCC_GPU_MEMNOC_GFX / 4] = 0x80000000;
    win[GPB_WIN_SMMU][GPB_SMMU_ID0 / 4] = 0x08;                     // 8 SMRs
    win[GPB_WIN_SMMU][GPB_SMMU_SMR(3) / 4] = 0x80000005;
    win[GPB_WIN_SMMU][GPB_SMMU_S2CR(3) / 4] = 0x1;
}

static void put16(uint8_t *p, unsigned v) {p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);}
static void put32(uint8_t *p, uint32_t v) {put16(p, v & 0xffff); put16(p + 2, v >> 16);}
static uint8_t db[GPB_CMD_DB_BYTES];

int main(void) {
    struct gpb_io io = {NULL, rd, wr, NULL};
    const unsigned all = (1u << GPB_WINDOWS) - 1;
    struct gpb_result r;

    // Mode 1 reads only, and stops at GPUCC while the cfg AHB clock is off.
    reset_model();
    gpb_run(&io, 1, all, &r);
    CHECK(writes == 0 && r.gcc == GPB_DONE && r.cc == GPB_SKIPPED_PREREQ && r.smmu == GPB_SKIPPED_PREREQ && r.gpu == GPB_SKIPPED_PREREQ);
    // Mode 2 enables the cfg AHB clock and reads GPUCC, nothing more.
    reset_model();
    gpb_run(&io, 2, all, &r);
    CHECK(r.enabled_ahb && r.cc == GPB_DONE && !gpb_gdsc_on(r.cx_gdscr) && r.smmu == GPB_SKIPPED_PREREQ && cx_writes == 0);
    // Mode 3 powers CX (one GDSCR write), turns on the SMMU's clocks and reads the SMMU's global space; GX stays off, so no GPU read.
    reset_model();
    gpb_run(&io, 3, all, &r);
    CHECK(cx_writes == 1 && r.powered_cx == 1 && r.powered_clocks && gpb_gdsc_on(r.cx_hwctrl));
    CHECK(r.smmu == GPB_DONE && r.smr_valid == 1 && r.smr_first == 3 && r.smr_first_val == 0x80000005 && r.s2cr_first_val == 1);
    CHECK(r.gpu == GPB_SKIPPED_PREREQ && !r.io_error);
    CHECK(win[GPB_WIN_GPUCC][GPB_CC_GX_GDSCR / 4] == 0x00222001);
    // CX already on: no GDSCR write.
    reset_model();
    win[GPB_WIN_GPUCC][GPB_CC_CX_GDSCR / 4] = GPB_GDSC_PWR_ON;
    win[GPB_WIN_GCC_GPU][GPB_GCC_GPU_CFG_AHB / 4] = 1;
    gpb_run(&io, 3, all, &r);
    CHECK(cx_writes == 0 && r.powered_cx == 2 && r.smmu == GPB_DONE);
    // An unmapped SMMU window is reported, not read.
    reset_model();
    gpb_run(&io, 3, all & ~(1u << GPB_WIN_SMMU), &r);
    CHECK(r.smmu == GPB_NOMAP);

    // Mode 6: votes, GMU RCG to GPLL0 div / 1.5, wake/sleep fields, the five branches.
    reset_model();
    gpb_run(&io, 3, all, &r);
    struct gpb_gmu_clock_on o;
    gpb_gmu_clocks_on(&io, &r, &o);
    CHECK(o.step == 5 && !o.failed);
    CHECK((o.vote & GPB_GPLL0_VOTES) == GPB_GPLL0_VOTES);
    CHECK((o.gmu_cfg & 0x71f) == ((6u << 8) | 2u) && !(o.gmu_cmd & GPB_RCG_ROOT_EN));
    CHECK(gpb_clock_on(o.cx_gmu) && (o.cx_gmu & GPB_CX_GMU_WAKE_SLEEP) == GPB_CX_GMU_WAKE_SLEEP);
    CHECK(gpb_clock_on(o.ahb) && gpb_clock_on(o.cxo) && gpb_clock_on(o.axi) && gpb_clock_on(o.memnoc));
    CHECK(gpb_gmu_regs_readable(&r, o.cx_gmu, o.ahb));
    // Without CX, mode 6 refuses before any write.
    reset_model();
    gpb_run(&io, 2, all, &r);
    unsigned before = writes;
    gpb_gmu_clocks_on(&io, &r, &o);
    CHECK(o.failed == 1 && o.step == 1 && writes == before);
    // An RCG whose root never turns on times out at step 2, before its configuration is written and before any branch.
    reset_model();
    gpb_run(&io, 3, all, &r);
    rcg_stuck = 1;
    gpb_gmu_clocks_on(&io, &r, &o);
    CHECK(o.step == 2 && o.failed == 1 && (o.gmu_cfg & 0x71f) == 0 && !gpb_clock_on(o.cx_gmu));

    // Command DB: slave 3 (ARC) with gfx.lvl, mx.lvl, cx.lvl and their level lists; slave 5 (BCM) with SH0.
    memset(db, 0, sizeof db);
    put32(db, 0x00000001); put32(db + 4, GPB_CMD_DB_MAGIC);
    put16(db + 8, 3); put16(db + 10, 0); put16(db + 12, 0x400); put16(db + 14, 3);
    put16(db + 24, 5); put16(db + 26, 0x100); put16(db + 28, 0x800); put16(db + 30, 1);
    static const char *const arc[3] = {"gfx.lvl", "mx.lvl", "cx.lvl"};
    for (unsigned i = 0; i < 3; ++i) {
        uint8_t *e = db + GPB_CMD_DB_DATA + 24 * i;
        memcpy(e, arc[i], strlen(arc[i]));
        put32(e + 16, 0x30000 + 0x10 * i); put16(e + 20, 6); put16(e + 22, (unsigned)(8 * i));
        uint8_t *aux = db + GPB_CMD_DB_DATA + 0x400 + 8 * i;
        put16(aux, 16); put16(aux + 2, 48); put16(aux + 4, 64 + i);
    }
    uint8_t *sh0 = db + GPB_CMD_DB_DATA + 0x100;
    memcpy(sh0, "SH0", 3); put32(sh0 + 16, 0x50000); put16(sh0 + 20, 4); put16(sh0 + 22, 0);
    struct gpb_cmd_db_entry e;
    CHECK(gpb_cmd_db_find(db, sizeof db, "gfx.lvl", &e) == 0 && e.address == 0x30000 && e.slave == 3 && e.aux_bytes == 6);
    CHECK(gpb_le16(e.aux) == 16 && gpb_le16(e.aux + 4) == 64);
    CHECK(gpb_cmd_db_find(db, sizeof db, "cx.lvl", &e) == 0 && e.address == 0x30020 && gpb_le16(e.aux + 4) == 66);
    CHECK(gpb_cmd_db_find(db, sizeof db, "SH0", &e) == 0 && e.address == 0x50000 && e.slave == 5);
    CHECK(gpb_cmd_db_find(db, sizeof db, "mx", &e) == 1);            // a prefix is not a match
    CHECK(gpb_cmd_db_find(db, sizeof db, "gmxc.lvl", &e) == 1);
    put32(db + 4, 0);
    CHECK(gpb_cmd_db_find(db, sizeof db, "gfx.lvl", &e) == -1);
    puts("PASS: pipa GPU power board (stages 1-3, GMU clocks, command DB)");
    return 0;
}
