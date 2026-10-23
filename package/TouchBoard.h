// SPDX-License-Identifier: BSD-2-Clause-Patent
// pipa (Xiaomi Pad 6, SM8250 "kona") touch controller board facts and the pure helpers around them, after UniXNU-sheng's
// TouchBoard.h (SM8550). Every constant states its source: DTS = the tablet's device tree (android-devinfo dump), gcc/pinctrl =
// Linux's gcc-sm8250.c / pinctrl-sm8250.c, elish = Linux's sm8250-xiaomi-elish dts (the Xiaomi Pad 5 Pro, the same SoC with its
// Novatek panel on the same SPI). Nothing here touches hardware; TouchProbe.cpp owns the MMIO.
#ifndef PIPA_TOUCH_BOARD_H
#define PIPA_TOUCH_BOARD_H
#include <stdint.h>

// DTS spi@990000 size 0x4000 (SE4 of the QUP wrapper qcom,qupv3_0_geni_se@9c0000, size 0x2000), clocks gcc 0x60/0x84/0x85.
#define TB_QUP_WRAPPER_BASE 0x009c0000ULL
#define TB_SE_BASE 0x00990000ULL
#define TB_SE_SIZE 0x4000ULL
// DTS: the wrapper's iommus = <&apps_smmu 0x5a3 0> (Android maps it, "fastmap"); apps-smmu@15000000 size 0x100000. Only needed if
// the engine's FIFO is disabled and transfers must go through its DMA.
#define TB_QUP_SMMU_SID 0x5a3
#define TB_APPS_SMMU_BASE 0x15000000ULL
#define TB_APPS_SMMU_SIZE 0x100000ULL
// gcc: GCC at 0x100000 (DTS qcom,gcc@100000), every QUP branch below votes in 0x52008, GPLL0 in 0x52018 bit 0.
#define TB_GCC_BASE 0x00100000ULL
#define TB_GCC_VOTE_ENABLE 0x52008
#define TB_GCC_GPLL0_ENABLE 0x52018
#define TB_GCC_GPLL0_MODE 0x0                 // Lucid PLL MODE, LOCK_DET bit 31
#define TB_VOTE_WRAP0_M_AHB (1u<<6)           // gcc_qupv3_wrap_0_m_ahb_clk, halt 0x17004
#define TB_VOTE_WRAP0_S_AHB (1u<<7)           // gcc_qupv3_wrap_0_s_ahb_clk, halt 0x17008 (hwcg bit 1)
#define TB_VOTE_WRAP0_CORE (1u<<8)            // gcc_qupv3_wrap0_core_clk, halt 0x23000
#define TB_VOTE_WRAP0_CORE_2X (1u<<9)         // gcc_qupv3_wrap0_core_2x_clk, halt 0x23008
#define TB_VOTE_WRAP0_S4 (1u<<14)             // gcc_qupv3_wrap0_s4_clk, halt 0x174cc
#define TB_VOTE_ALL (TB_VOTE_WRAP0_CORE_2X|TB_VOTE_WRAP0_CORE|TB_VOTE_WRAP0_M_AHB|TB_VOTE_WRAP0_S_AHB|TB_VOTE_WRAP0_S4)
#define TB_HALT_M_AHB 0x17004
#define TB_HALT_S_AHB 0x17008
#define TB_HALT_S4 0x174cc
#define TB_HALT_CORE 0x23000
#define TB_HALT_CORE_2X 0x23008
#define TB_CBCR_CLK_OFF (1u<<31)
// gcc_qupv3_wrap0_s4_clk_src: cmd_rcgr 0x174d0, mnd 16, hid 5, gcc_parent_map_0, registered for DFS (clk-rcg2.c SE_* offsets).
#define TB_S4_RCG 0x174d0
#define TB_RCG_CMD 0x0
#define TB_RCG_CFG 0x4
#define TB_RCG_DFSR_CMD 0x14                  // bit0 DFS enable, bits 4:1 current level
#define TB_RCG_PERF_CFG(l) (0x1c+4*(l))
#define TB_RCG_PERF_M(l) (0x5c+4*(l))
#define TB_RCG_PERF_N(l) (0x9c+4*(l))
#define TB_DFS_LEVELS 8
// TLMM: DTS pinctrl@f000000 (one 16 MiB window), pinctrl-sm8250.c three tiles: west +0x100000, south +0x500000, north +0x900000;
// in a tile ctl_reg = 0x1000*gpio (pull 1:0, func 5:2, drive 8:6 as mA/2-1, OE 9), io_reg +4 (bit 0 in, bit 1 out), intr_cfg +8,
// intr_status +0xc.
#define TB_TLMM_BASE 0x0f000000ULL
#define TB_TILE_SOUTH 0x500000u
#define TB_TILE_NORTH 0x900000u
#define TB_GPIO_STRIDE 0x1000
#define TB_GPIO_SPI_FIRST 8                   // DTS qupv3_se4_spi_active: gpio8/9/10 (data and clock), ts_cs_active: gpio11; all NORTH
#define TB_GPIO_SPI_CS 11
#define TB_GPIO_IRQ 39                        // DTS novatek,irq-gpio = <&tlmm 39 0x2001> (rising edge); SOUTH
#define TB_FUNC_QUP4 1                        // pinctrl-sm8250.c PINGROUP(8..11, NORTH, qup4, ...): function slot 1
#define TB_DRIVE_6MA 2                        // DTS drive-strength = <6>, bias-disable
// The vendor driver sets SPI_MODE_0 (nt36xxx nvt_ts_probe); DTS spi-max-frequency = <9600000> (elish runs its panel at 19.2 MHz).
#define TB_SPI_MODE 0
#define TB_SPI_DEFAULT_HZ 9600000u
#define TB_SPI_MAX_HZ 19200000u

static inline uint32_t tb_pin_cfg(unsigned func, unsigned drive_code) {return (func&15u)<<2|(drive_code&7u)<<6;}   // pull disabled
static inline uint32_t tb_tlmm_tile(unsigned gpio) {return gpio==TB_GPIO_IRQ?TB_TILE_SOUTH:TB_TILE_NORTH;}   // the pins used here
static inline uint32_t tb_tlmm_ctl(unsigned gpio) {return tb_tlmm_tile(gpio)+(uint32_t)gpio*TB_GPIO_STRIDE;}
static inline uint32_t tb_tlmm_io(unsigned gpio) {return tb_tlmm_ctl(gpio)+4;}

// Source clock rates by RCG source-select value (gcc_parent_map_0: 0 = TCXO, 1 = GPLL0 main, 6 = GPLL0 even). GPLL0 main is
// 600 MHz and even 300 MHz (the frequency table: 32 MHz = even * 8 / 75).
static inline uint64_t tb_src_hz(unsigned sel) {return sel==0?19200000ULL:sel==1?600000000ULL:sel==6?300000000ULL:0;}
// clk-rcg2.c calc_rate: half-integer divider, then M/N.
static inline uint64_t tb_calc_rate(uint64_t prate, uint32_t m, uint32_t n, unsigned mode, unsigned hid) {
    if (hid) {prate*=2;prate/=hid+1;}
    if (mode && n) {prate*=m;prate/=n;}
    return prate;
}
// Frequency of DFS level l (or of the plain RCG) from its CFG (src 10:8, mode 13:12, hid 4:0), M register and N register (stored
// inverted: n = (~N & mask) + m, mnd_width 16).
static inline uint64_t tb_dfs_level_hz(uint32_t cfg, uint32_t mreg, uint32_t nreg) {
    unsigned src=(cfg>>8)&7,mode=(cfg>>12)&3,hid=cfg&0x1f;
    uint32_t m=0,n=0;
    if (mode) {m=mreg&0xffff;n=((~nreg)&0xffff)+m;}
    return tb_calc_rate(tb_src_hz(src),m,n,mode,hid);
}
struct tb_clock_choice {unsigned level, div;uint64_t sclk_hz, actual_hz;};
// Choose the level that gives the highest SPI rate not above target with divider div = ceil(sclk / (os * target)). Returns 0 on
// success.
static inline int tb_choose_clock(const uint64_t *level_hz, unsigned n, unsigned os, unsigned target, struct tb_clock_choice *out) {
    int found=0;
    for (unsigned l=0;l<n;++l) {
        uint64_t f=level_hz[l];if (!f) continue;
        uint64_t d=((f+(uint64_t)os*target-1)/((uint64_t)os*target));if (d<1) d=1;if (d>0xfff) continue;
        uint64_t actual=f/(os*d);if (actual>target) continue;
        if (!found || actual>out->actual_hz || (actual==out->actual_hz && d<out->div)) {
            out->level=l;out->div=(unsigned)d;out->sclk_hz=f;out->actual_hz=actual;found=1;
        }
    }
    return found?0:-1;
}
#endif
