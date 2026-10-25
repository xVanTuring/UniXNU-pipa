// SPDX-License-Identifier: BSD-2-Clause-Patent
// pipa's power and volume keys for the HID input (drivers/NativeHidInput: native_buttons_set_sink, report 2), `pipa-buttons=1`.
// A kernel thread reads the PM8150 PON's and GPIO6's RT_STS registers through the SPMI arbiter's observer channels every 10 ms
// (reads only; nothing in the PMIC is configured), debounces over two reads and passes every change of the pressed set to the
// sink. The keys and registers are in ButtonsCore.h. The arbiter is the one BootRamoops.c already uses for the restart (version
// 5, EE 0); that file keeps its own copy of the read for its write path, which runs once at boot_progress_start and at the
// restart. Built with the kernel's C flags into the display archive (build-native-legacy-display.py), started by
// LegacyDisplayProbe.cpp's late probe. Not yet run on the tablet.
#include <stdint.h>
#include "ButtonsCore.h"

extern uint64_t ml_io_map(uint64_t phys_addr, uint64_t size);
extern int PE_parse_boot_argn(const char *arg_string, void *arg_ptr, int max_arg);
extern int kernel_thread_start(void (*continuation)(void *, int), void *parameter, void **new_thread);
extern void thread_deallocate(void *thread);
extern void IOSleep(unsigned milliseconds);
extern int snprintf(char *, unsigned long, const char *, ...);

#define SPMI_CORE 0x0C440000ull
#define SPMI_OBSRVR 0x0E600000ull
#define SPMI_APID_MAP 0x900u           // core + 4 * APID: bits 19:8 the PPID
#define SPMI_MAX_APIDS 512u
#define SPMI_OBS_APID_STRIDE 0x80ull   // EE 0
#define SPMI_CMD 0x00u
#define SPMI_STATUS 0x08u
#define SPMI_RDATA0 0x18u
#define SPMI_POLLS 100000u
#define POLL_MS 10u

// The log (BootRamoops.c): one row, rewritten on every change, and the change count.
char native_buttons_row[120];
volatile uint32_t native_buttons_changes, native_buttons_mask;
volatile int native_buttons_error;

static void (*volatile sink)(void *, uint32_t);
static void *volatile sink_context;

// Called by NativeHidInput when it starts (and with nullptr when it stops). The context is stored first, so a reader that sees
// the sink sees its context.
void native_buttons_set_sink(void (*callback)(void *, uint32_t), void *context) {
    __atomic_store_n(&sink, (void (*)(void *, uint32_t))0, __ATOMIC_RELEASE);
    __atomic_store_n(&sink_context, context, __ATOMIC_RELEASE);
    __atomic_store_n(&sink, callback, __ATOMIC_RELEASE);
}

static volatile uint32_t *observer_for(volatile uint32_t *core, uint32_t apids, uint32_t ppid) {
    for (uint32_t i = 0; i < apids; ++i) {
        uint32_t map = core[(SPMI_APID_MAP + 4 * i) / 4];
        if (!map || ((map >> 8) & 0xFFF) != ppid) continue;
        uint64_t phys = SPMI_OBSRVR + SPMI_OBS_APID_STRIDE * i;
        uint64_t page = ml_io_map(phys & ~0xFFFull, 0x1000);
        return page ? (volatile uint32_t *)(uintptr_t)(page + (phys & 0xFFF)) : 0;
    }
    return 0;
}
// One byte of the peripheral (BootRamoops.c's spmi_read8). 0 = done, 1 = no answer, 2 = an error status.
static int read8(volatile uint32_t *observer, uint8_t address, uint8_t *value) {
    observer[SPMI_CMD / 4] = (1u << 27) | ((uint32_t)address << 4);
    for (uint32_t poll = 0; poll < SPMI_POLLS; ++poll) {
        uint32_t status = observer[SPMI_STATUS / 4];
        if (status & 1) {
            if (status & 0xE) return 2;
            *value = (uint8_t)observer[SPMI_RDATA0 / 4];
            return 0;
        }
    }
    return 1;
}

static void buttons_thread(void *parameter, int wait_result) {
    (void)parameter; (void)wait_result;
    volatile uint32_t *core = (volatile uint32_t *)(uintptr_t)ml_io_map(SPMI_CORE, 0x2000);
    if (!core) {native_buttons_error = -1; return;}
    uint32_t version = core[0], apids = core[1] & 0x7FF;
    if (version < 0x50000000u || version >= 0x70000000u || !apids || apids > SPMI_MAX_APIDS) {native_buttons_error = -2; return;}
    volatile uint32_t *pon = observer_for(core, apids, BTN_PON_PPID), *gpio = observer_for(core, apids, BTN_GPIO6_PPID);
    if (!pon || !gpio) {native_buttons_error = pon ? -4 : -3; return;}
    // What the firmware left GPIO6 as: an input with the pull-up enabled reads 0x10 / mode 0 / pull 0 / enable 0x80.
    uint8_t type = 0, subtype = 0, mode = 0, pull = 0, enable = 0, pon_sts = 0, gpio_sts = 0;
    int rc = read8(gpio, BTN_GPIO_TYPE, &type) | read8(gpio, BTN_GPIO_SUBTYPE, &subtype) | read8(gpio, BTN_GPIO_MODE_CTL, &mode) |
             read8(gpio, BTN_GPIO_DIG_PULL, &pull) | read8(gpio, BTN_GPIO_EN_CTL, &enable) | read8(pon, BTN_RT_STS, &pon_sts) |
             read8(gpio, BTN_RT_STS, &gpio_sts);
    snprintf(native_buttons_row, sizeof native_buttons_row,
             "start rc%d gpio6 type%02x/%02x mode%02x pull%02x en%02x sts%02x pon sts%02x", rc, type, subtype, mode, pull, enable,
             gpio_sts, pon_sts);
    native_buttons_changes = 1;
    if (rc) {native_buttons_error = -5; return;}
    struct buttons_debounce d = {0, 0, 0};
    uint32_t errors = 0;
    // pipa-power-at=<seconds>: one synthesized power press this long after the probe starts, through the same sink the real key uses,
    // so a round can leave the tablet with the display off (iPadOS locks on a power press) instead of burning the panel.
    unsigned powerAt = 0;
    PE_parse_boot_argn("pipa-power-at", &powerAt, sizeof powerAt);
    uint32_t ticks = 0;
    unsigned powerSent = 0;
    for (;;) {
        IOSleep(POLL_MS);
        if (powerAt && !powerSent && ++ticks >= powerAt * (1000u / POLL_MS)) {
            powerSent = 1;
            void (*cb)(void *, uint32_t) = __atomic_load_n(&sink, __ATOMIC_ACQUIRE);
            void *ctx = __atomic_load_n(&sink_context, __ATOMIC_ACQUIRE);
            if (cb) {
                cb(ctx, BTN_POWER);
                IOSleep(150);
                cb(ctx, 0);
            }
            snprintf(native_buttons_row, sizeof native_buttons_row, "power press at %u s%s", powerAt, cb ? "" : " (no sink)");
            native_buttons_changes = native_buttons_changes + 1;
        }
        if (read8(pon, BTN_RT_STS, &pon_sts) || read8(gpio, BTN_RT_STS, &gpio_sts)) {++errors; continue;}
        if (!buttons_debounce_step(&d, buttons_mask(pon_sts, gpio_sts), 2)) continue;
        native_buttons_mask = d.stable;
        void (*callback)(void *, uint32_t) = __atomic_load_n(&sink, __ATOMIC_ACQUIRE);
        if (callback) callback(__atomic_load_n(&sink_context, __ATOMIC_ACQUIRE), d.stable);
        snprintf(native_buttons_row, sizeof native_buttons_row, "keys %s%s%s(mask %x)%s, pon %02x gpio6 %02x, read errors %u",
                 (d.stable & BTN_POWER) ? "power " : "", (d.stable & BTN_VOLUME_UP) ? "volume-up " : "",
                 (d.stable & BTN_VOLUME_DOWN) ? "volume-down " : "", d.stable, callback ? "" : " no sink", pon_sts, gpio_sts, errors);
        native_buttons_changes = native_buttons_changes + 1;
    }
}

void native_buttons_probe(void) {
    static int started;
    unsigned on = 0;
    if (started || !PE_parse_boot_argn("pipa-buttons", &on, sizeof on) || !on) return;
    started = 1;
    void *thread = 0;
    if (kernel_thread_start(buttons_thread, 0, &thread) == 0) thread_deallocate(thread);
    else native_buttons_error = -12;
}
