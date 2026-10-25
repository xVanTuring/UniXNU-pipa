// SPDX-License-Identifier: BSD-2-Clause-Patent
// pipa's battery: the PM8150B's gen4 fuel gauge and SMB5 charger, and the pure decoding around them for NativeBattery.cpp;
// host-tested by NativeBatteryTests.c. Sources: the tablet's device tree (uefi-pipa Resources/DTBs/pipa.dts, qcom,pm8150b@2
// with qpnp,fg@4000..4300 and qcom,qpnp-smb5@1000..1700) and Linux's qpnp-fg-gen4.c, fg-reg.h, fg-memif.c, fg-util.c and
// smb5-lib.c, smb5-reg.h (LineageOS android_kernel_xiaomi_sm8250, lineage-20).
// SPMI PPID = (USID << 8) | (peripheral address >> 8); the gauge and the charger are both on PM8150B, USID 2.
//   SOC       0x240 fg-batt-soc   0x09 MSOC and 0x0A its copy: 0..255, 255 = full
//   VBATT     0x241 fg-batt-info  0xA0/0xA1 and 0xA6/0xA7 (copy): uV = raw * 122070 / 1000
//   IBATT     0x241 fg-batt-info  0xA2/0xA3 and 0xA8/0xA9 (copy): uA = sign15(raw) * 488281 / 1000
//   BATT_TEMP 0x241 fg-batt-info  0x50/0x51 (10-bit, 0.25 C) — the fallback; the gauge proper keeps it in SRAM word 328
//   stage     0x210 SMB5 CHGR    0x06 bits 2:0 (1 trickle, 2 pre, 3 full-on, 4 taper, 5 terminate, 7 disabled)
//   chg_en    0x210 SMB5 CHGR    0x42 bit 0
//   external  0x213 SMB5 USBIN   0x10 bit 4, VBUS present (smblib_get_prop_usb_present)
//   present   0x212 SMB5 BATIF   0x10 bits 5:4, battery terminal or therm/ID missing
//   wd config 0x216 SMB5 MISC    0x53 bark/bite timeouts, read only (nothing is petted)
#ifndef PIPA_NATIVE_BATTERY_BOARD_H
#define PIPA_NATIVE_BATTERY_BOARD_H
#include <stdint.h>

#define BAT_PPID_FG_SOC 0x240u
#define BAT_PPID_FG_INFO 0x241u
#define BAT_PPID_FG_RRADC 0x242u
#define BAT_PPID_FG_MEMIF 0x243u
#define BAT_PPID_SMB_CHGR 0x210u
#define BAT_PPID_SMB_DCDC 0x211u
#define BAT_PPID_SMB_BATIF 0x212u
#define BAT_PPID_SMB_USBIN 0x213u
#define BAT_PPID_SMB_MISC 0x216u

#define BAT_FG_MSOC 0x09u
#define BAT_FG_MSOC_CP 0x0Au
#define BAT_FG_VBATT_LSB 0xA0u
#define BAT_FG_VBATT_LSB_CP 0xA6u
#define BAT_FG_IBATT_LSB 0xA2u
#define BAT_FG_IBATT_LSB_CP 0xA8u
#define BAT_FG_BATT_TEMP_LSB 0x50u
#define BAT_FG_TEMP_SRAM_WORD 328u
#define BAT_FG_SRAM_RD_DATA0 0x67u
#define BAT_FG_MEM_INTF_CFG 0x50u
#define BAT_FG_IMA_CTL 0x51u
#define BAT_FG_IMA_OPR_STS 0x54u
#define BAT_FG_IMA_EXP_STS 0x55u
#define BAT_FG_IMA_HW_STS 0x56u
#define BAT_FG_IMA_ERR_STS 0x5Fu
#define BAT_FG_ADDR_LSB 0x61u
#define BAT_FG_MEM_GNT 0x80u
#define BAT_FG_MEM_IACS_SLCT 0x20u
#define BAT_FG_IACS_RDY 0x02u

#define BAT_SMB_CHG_STAGE 0x06u
#define BAT_SMB_CHG_ENABLE 0x42u
#define BAT_SMB_USBIN_RT_STS 0x10u
#define BAT_SMB_BATIF_RT_STS 0x10u
#define BAT_SMB_WDOG_CFG 0x53u
#define BAT_SMB_STAGE_TRICKLE 1u
#define BAT_SMB_STAGE_TAPER 4u
#define BAT_SMB_STAGE_TERMINATE 5u
#define BAT_SMB_USBIN_PLUGIN (1u << 4)
#define BAT_SMB_BATT_MISSING ((1u << 5) | (1u << 4))

// The gauge's full-scale SOC and the pack's design capacity (Android's /sys/class/power_supply/bms:
// CHARGE_FULL_DESIGN = 8840000 uAh, battery type M82_sunwoda_8840mah).
#define BAT_SOC_FULL 255u
#define BAT_DESIGN_MAH 8840u
// IOPMPowerSource TimeRemaining is in minutes; 0xFFFF means unknown.
#define BAT_TIME_UNKNOWN 0xFFFFu

static inline unsigned battery_percent(unsigned raw) {
    if (raw >= BAT_SOC_FULL) return 100;
    if (!raw) return 0;
    return ((raw - 1) * 98 + 126) / 253 + 1;
}

static inline int battery_voltage_uv(unsigned raw) {
    return (int)((uint64_t)raw * 122070u / 1000u);
}

static inline int battery_current_ua(unsigned raw) {
    int v = (int)(raw & 0x7FFFu);
    if (raw & 0x8000u) v -= 0x8000;
    return (int)(((int64_t)v * 488281) / 1000);
}

// The gauge's 10-bit battery temperature: signed, 0.25 C per LSB (fg_gen4_get_battery_temp's sign_extend32(buf, 9)
// * 100 / 40 deci-degrees; the SRAM word is 16 bits with the field in bits 9:0).
static inline int battery_temp_millic(unsigned raw) {
    int v = (int)(raw & 0x3FFu);
    if (raw & 0x200u) v -= 0x400;
    return v * 250;
}

// The charger's own state machine is charging in trickle, pre, full-on and taper; terminate, pause, inhibit and disable are not.
// (CHARGING_ENABLE_CMD is a command register that the vendor driver only writes, so the stage, not its read-back, decides.)
static inline int battery_charging(unsigned stage) {
    return stage >= BAT_SMB_STAGE_TRICKLE && stage <= BAT_SMB_STAGE_TAPER;
}

static inline uint16_t battery_u16(const uint8_t *bytes) {
    return (uint16_t)(bytes[0] | bytes[1] << 8);
}
#endif
