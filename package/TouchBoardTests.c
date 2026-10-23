// Host test for pipa's TouchBoard.h helpers and the SMMU pass-through plan against values read on the tablet or computed by hand
// from the cited Linux tables. cc -std=c11 -Wall -Wextra -Werror package/TouchBoardTests.c && ./a.out
#include "TouchBoard.h"
#include "SmmuBypass.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(c) do{if(!(c)){printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c);exit(1);}}while(0)

// The apps SMMU as NativeUfs's survey read it on pipa (2026-10-23): IDR0 0x4c017e57 (87 stream-match groups), IDR1 0x6000004a (74
// context banks, 128 global pages), IDR2 0x00005111; UEFI's UFS streams 0xe0 -> bank 3 and 0x4e0 -> bank 2 among 7 valid groups.
struct fake {uint32_t reg[0x100000 / 4]; unsigned writes;};
static uint32_t frd(void *c, unsigned o) {return ((struct fake *)c)->reg[o / 4];}
static void fwr(void *c, unsigned o, uint32_t v) {((struct fake *)c)->reg[o / 4] = v; ((struct fake *)c)->writes++;}
static struct fake f;
static void pipa_smmu(void) {
    memset(&f, 0, sizeof f);
    f.reg[0x20 / 4] = 0x4c017e57; f.reg[0x24 / 4] = 0x6000004a; f.reg[0x28 / 4] = 0x00005111;
    static const uint32_t smr[7] = {0x80000c00, 0x80000c01, 0x800004e0, 0x800000e0, 0x80001000, 0x80001001, 0x80001002};
    static const uint32_t s2cr[7] = {0, 1, 2, 3, 4, 5, 6};
    for (unsigned i = 0; i < 7; ++i) {f.reg[0x800 / 4 + i] = smr[i]; f.reg[0xc00 / 4 + i] = s2cr[i];}
    for (unsigned b = 0; b < 7; ++b) {f.reg[0x1000 / 4 + b] = 0x1f000; f.reg[0x1800 / 4 + b] = 1; f.reg[(128 + b) * 1024] = 0x009f00e0;}
}

int main(void) {
    // TLMM: the SPI pins in the north tile, the attention line in the south one (pinctrl-sm8250.c PINGROUP tiles).
    CHECK(TB_TLMM_BASE + tb_tlmm_ctl(8) == 0x0f908000ULL && TB_TLMM_BASE + tb_tlmm_ctl(11) == 0x0f90b000ULL);
    CHECK(TB_TLMM_BASE + tb_tlmm_ctl(TB_GPIO_IRQ) == 0x0f527000ULL && TB_TLMM_BASE + tb_tlmm_io(TB_GPIO_IRQ) == 0x0f527004ULL);
    CHECK(tb_pin_cfg(TB_FUNC_QUP4, TB_DRIVE_6MA) == ((1u << 2) | (2u << 6)));
    // Clock sources: 100 MHz = GPLL0 main / 6 (hid 11), 75 MHz = GPLL0 even / 4 (src 6, hid 7), 19.2 MHz = TCXO,
    // 7.3728 MHz = even * 384 / 15625 (dual-edge M/N mode 2).
    CHECK(tb_dfs_level_hz((1u << 8) | 11, 0, 0) == 100000000ULL);
    CHECK(tb_dfs_level_hz((6u << 8) | 7, 0, 0) == 75000000ULL);
    CHECK(tb_dfs_level_hz(0, 0, 0) == 19200000ULL);
    CHECK(tb_dfs_level_hz((6u << 8) | (2u << 12), 384, (~(15625u - 384u)) & 0xffff) == 7372800ULL);
    // 9.6 MHz (the DT's rate) from TCXO / 2 with oversampling 1; 19.2 MHz from TCXO / 1.
    const uint64_t levels[8] = {19200000, 32000000, 48000000, 64000000, 75000000, 80000000, 96000000, 100000000};
    struct tb_clock_choice c = {0};
    CHECK(!tb_choose_clock(levels, 8, 1, TB_SPI_DEFAULT_HZ, &c) && c.actual_hz == 9600000 && c.level == 0 && c.div == 2);
    CHECK(!tb_choose_clock(levels, 8, 1, TB_SPI_MAX_HZ, &c) && c.actual_hz == 19200000 && c.div == 1);
    // The QUP wrapper's stream on pipa's SMMU: a fresh group (the first unused, 7) on the top bank (73), and a second plan finds it.
    pipa_smmu();
    struct smmu_io io = {&f, frd, fwr};
    struct smmu_plan p = smmu_plan_bypass(&io, TB_QUP_SMMU_SID, 0x1ffc20000ULL, 0x20000);
    CHECK(!p.error && p.slot == 7 && p.bank == 73 && p.pages == 128 && !p.reuse);
    CHECK(!smmu_configure_bypass(&io, &p, TB_QUP_SMMU_SID));
    CHECK(f.reg[0x800 / 4 + 7] == (0x80000000u | TB_QUP_SMMU_SID) && f.reg[0xc00 / 4 + 7] == 73);
    CHECK(f.reg[0x1000 / 4 + 73] == 0x10000 && f.reg[0x1800 / 4 + 73] == 1 && f.reg[(128 + 73) * 1024] == 0);
    // UEFI's UFS mappings are left alone.
    CHECK(f.reg[0x800 / 4 + 3] == 0x800000e0 && f.reg[0xc00 / 4 + 3] == 3 && f.reg[0x800 / 4 + 2] == 0x800004e0);
    p = smmu_plan_bypass(&io, TB_QUP_SMMU_SID, 0x1ffc20000ULL, 0x20000);
    CHECK(!p.error && p.reuse && p.slot == 7);
    // XNU's memory on pipa ends at 0x980000000 + 0x80000000: inside the 36-bit width; beyond it is refused.
    CHECK(smmu_plan_bypass(&io, TB_QUP_SMMU_SID, 1ULL << 36, 0x1000).error);
    puts("PASS: pipa touch board (TLMM tiles, clocks, SMMU pass-through on the tablet's SMMU)");
    return 0;
}
