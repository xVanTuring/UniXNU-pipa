// SPDX-License-Identifier: BSD-2-Clause-Patent
// pipa's three keys and the pure decoding around them, for Buttons.c; host-tested by ButtonsTests.c. Sources: the tablet's
// device tree (uefi-pipa Resources/DTBs/pipa.dts) and Linux's pm8941-pwrkey.c / pinctrl-spmi-gpio.c.
//   power       PM8150 (SID 0) PON at 0x800, pon-type 0 "kpdpwr" (KEY_POWER): PON_RT_STS 0x810 bit 0, set while pressed
//   volume down the same PON, pon-type 1 "resin" (KEY_VOLUMEDOWN): PON_RT_STS bit 1, set while pressed
//   volume up   gpio_keys vol_up = PM8150 GPIO6 (pinctrl@c000, peripheral 0xc5), active low with a pull-up (key_vol_up_default):
//               its RT_STS 0xc510 bit 0 is the input level, 0 while pressed
// The mask is NativeHidInput's report 2 (Consumer): bit 1 Power, bit 2 Volume Increment, bit 3 Volume Decrement.
#ifndef PIPA_BUTTONS_CORE_H
#define PIPA_BUTTONS_CORE_H
#include <stdint.h>

#define BTN_PON_PPID 0x008u
#define BTN_GPIO6_PPID 0x0c5u
#define BTN_RT_STS 0x10u
#define BTN_PON_KPDPWR (1u << 0)
#define BTN_PON_RESIN (1u << 1)
#define BTN_GPIO_IN (1u << 0)
// PMIC GPIO configuration registers (pinctrl-spmi-gpio.c), logged once to show whether the firmware left GPIO6 an input.
#define BTN_GPIO_TYPE 0x04u                // 0x10 = GPIO
#define BTN_GPIO_SUBTYPE 0x05u
#define BTN_GPIO_MODE_CTL 0x40u            // bits 6:4 (LV/MV) mode, 0 = digital input
#define BTN_GPIO_DIG_PULL 0x42u            // 0 = 30 uA pull-up
#define BTN_GPIO_EN_CTL 0x46u              // bit 7 enable

#define BTN_POWER (1u << 1)
#define BTN_VOLUME_UP (1u << 2)
#define BTN_VOLUME_DOWN (1u << 3)

static inline uint32_t buttons_mask(uint8_t pon_rt_sts, uint8_t gpio6_rt_sts) {
    return ((pon_rt_sts & BTN_PON_KPDPWR) ? BTN_POWER : 0) | ((pon_rt_sts & BTN_PON_RESIN) ? BTN_VOLUME_DOWN : 0) |
           ((gpio6_rt_sts & BTN_GPIO_IN) ? 0 : BTN_VOLUME_UP);
}

// A new mask counts once it has been read the same `samples` times in a row (the DT's debounce: 15 ms for volume up; the PON
// debounces its own keys in hardware). Returns 1 when the stable mask changed.
struct buttons_debounce {uint32_t stable, candidate; unsigned seen;};
static inline int buttons_debounce_step(struct buttons_debounce *d, uint32_t mask, unsigned samples) {
    if (mask != d->candidate) {d->candidate = mask; d->seen = 1;}
    else if (d->seen < samples) ++d->seen;
    if (d->seen >= samples && d->candidate != d->stable) {d->stable = d->candidate; return 1;}
    return 0;
}
#endif
