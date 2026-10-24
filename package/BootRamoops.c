// SPDX-License-Identifier: BSD-2-Clause-Patent
// The kernel's log in the stock ramoops console zone, after the firmware's (uefi-pipa's RamoopsLogLib writes the same zone): the
// boot_progress_* hooks of zzz-boot-progress.patch (XNU_BOOT_PROGRESS) without a progress page. Stages and process events are
// written as they come; from boot_progress_start on, a thread appends the message buffer's new text every 100 ms. The zone
// survives the warm reset the firmware makes the power key's long press (and power with volume down) do, and the log reader
// (package/linux-log, scripts/read-pipa-log.py) reads it back as console-ramoops-0. The console stays XNU's (no takeover), so the
// panic screen is unchanged. The panic screen (scripts/xnu-panic-screen.c) copies the first panic's formatted message into
// boot_progress_panic_text, then calls boot_progress_panic_hook with interrupts masked: with one CPU running (the ADT's
// cpu_regs) no thread runs after that, so the hook itself appends the message buffer's rest and the panic text.
//
// The tablet restarts into fastboot by itself, with a warm reset that keeps the zone: pipa-restart-after=<s> seconds after
// boot_progress_start (0 = never), pipa-panic-restart-after=<s> (default 10) after a panic, and after pipa-restart-crashes=<n>
// crashes of SpringBoard or backboardd (0 = off), so a round that only repeats a UI crash ends early. The restart is uefi-pipa's
// RamoopsLogLib RestartIntoFastboot, which the firmware verified on this tablet: the PM8150 PON's reboot reason (bootloader) and
// its warm reset types through the SPMI arbiter (done once at boot_progress_start, from a thread), then at the restart the IMEM
// restart reason, the two SCM calls (secure watchdog debug boot part off, PMIC arbiter halted) and PS_HOLD deasserted, with
// PSCI SYSTEM_RESET as the fallback.
//
// The zone: 2 MiB at 0xB0000000 (the stock kernel's ramoops_memreserve=4M: console then pmsg, no ECC), a persistent_ram_buffer:
// the signature "DBGC", the write position and the used size, then the ring. Built without XNU's headers (scripts/build-boot-progress.py),
// so the few kernel interfaces it calls are declared here.

typedef unsigned long u64;
typedef unsigned int u32;
typedef unsigned char u8;

#define ZONE_BASE  0xB0000000UL
#define ZONE_SIZE  0x200000UL
#define ZONE_SIG   0x43474244U                 // "DBGC"
#define ZONE_HEAD  12UL
#define ZONE_DATA  (ZONE_SIZE - ZONE_HEAD)
#define EARLY_SIZE 8192U
#define PASS_MS    100U

// The restart (uefi-pipa RamoopsLogLib.c): the version 5 SPMI arbiter of the pipa device tree (qcom,spmi@c440000, EE 0) and the
// PM8150 PON at SID 0 peripheral 0x08.
#define SPMI_CORE             0x0C440000UL
#define SPMI_CHNLS            0x0C600000UL
#define SPMI_OBSRVR           0x0E600000UL
#define SPMI_CNFG             0x0C40A000UL
#define SPMI_EE               0U
#define SPMI_APID_MAP         0x900U          // core + 4 * APID: bits 19:8 the PPID
#define SPMI_CNFG_OWNER       0x700U          // cnfg + 4 * APID: bits 2:0 the EE that may write it
#define SPMI_MAX_APIDS        512U
#define SPMI_OBS_APID_STRIDE  0x80UL
#define SPMI_CHNL_APID_STRIDE 0x10000UL
#define SPMI_CMD              0x00U
#define SPMI_STATUS           0x08U
#define SPMI_WDATA0           0x10U
#define SPMI_RDATA0           0x18U
#define SPMI_POLLS            100000U
#define PON_PPID              0x008U
#define PON_KPDPWR_S2_CTL     0x42
#define PON_COMBO_S2_CTL      0x4A
#define PON_PS_HOLD_RST_CTL   0x5A
#define PON_SOFT_RB_SPARE     0x8F
#define PON_RESET_EN          0x80
#define PON_REASON_BOOTLOADER (2 << 1)        // SOFT_RB_SPARE bits 7:1
#define PS_HOLD               0x0C264000UL    // restart@c264000
#define IMEM_RESTART_REASON   0x146BF65CUL    // qcom,msm-imem-restart_reason
#define RESTART_BOOTLOADER    0x77665500U
#define SCM_WDOG_DEBUG_OFF    0xC2000109UL
#define SCM_HALT_PMIC_ARB     0xC2000901UL
#define PSCI_SYSTEM_RESET     0x84000009UL

extern u64 ml_io_map(u64 phys_addr, u64 size);
extern int PE_parse_boot_argn(const char *arg_string, void *arg_ptr, int max_arg);
extern int kernel_thread_start(void (*continuation)(void *, int), void *parameter, void **new_thread);
extern void thread_deallocate(void *thread);
extern void IOSleep(unsigned milliseconds);
extern int snprintf(char *, unsigned long, const char *, ...);
struct msgbuf {int msg_magic, msg_size, msg_bufx, msg_bufr; char *msg_bufc;};
extern struct msgbuf *msgbufp;

void boot_progress_init(void);
void boot_progress_start(void);
int boot_progress_active(void);
void boot_progress_bind(unsigned long, unsigned, unsigned, unsigned, unsigned);
void boot_progress_putc(char);
void boot_progress_stage(const char *);
void boot_progress_irq(u64, u64, u64);
void boot_progress_poll(void);
void boot_progress_process_stage(int, const char *, const char *);
void boot_progress_exit(int, const char *, int, unsigned, u64);
void boot_progress_panic_hook(void);
int boot_progress_prepare_scanout(void);
void boot_progress_cancel_scanout(void);
int boot_progress_pulse_anchor(unsigned long, u64, unsigned, unsigned, unsigned);

static volatile u8 *zone;                       // the zone's mapping, once boot_progress_start made it
static char early[EARLY_SIZE];                  // what came before it
static u32 early_len;
static volatile int lock;
static char last_byte = '\n';                   // the zone's last byte written, for line()
static u32 user_faults;
static volatile u32 *ps_hold, *imem;            // the restart's registers, mapped by boot_progress_start
static u32 restart_after, panic_restart_after = 10, restart_crashes;
static volatile u32 ui_crashes;                 // SpringBoard and backboardd exits by a crash signal
// Written by the panic screen (patch-xnu-entry-probe.py resolves the symbol), 256 bytes, the first panic's message.
char boot_progress_panic_text[256];

// The lock masks interrupts while it is held, so the holder is never preempted: on one CPU a spinning taker would otherwise
// wait for a holder that cannot run.
static u64 take(void) {
  u64 daif;
  __asm__ volatile("mrs %0, daif\n\tmsr daifset, #3" : "=r"(daif) :: "memory");
  while (__atomic_exchange_n(&lock, 1, __ATOMIC_ACQUIRE)) {}
  return daif;
}
static void give(u64 daif) {
  __atomic_store_n(&lock, 0, __ATOMIC_RELEASE);
  __asm__ volatile("msr daif, %0" :: "r"(daif) : "memory");
}

// Bytes into the ring, the oldest going once it is full: RamoopsLogLib's AppendByte, a run at a time.
static void zone_put(const char *text, unsigned n) {
  volatile u32 *head = (volatile u32 *)zone;
  u32 start = head[1], size = head[2];
  if (head[0] != ZONE_SIG || start >= ZONE_DATA || size > ZONE_DATA) {head[0] = ZONE_SIG; start = 0; size = 0;}
  for (unsigned i = 0; i < n; ++i) {
    zone[ZONE_HEAD + start] = (u8)text[i];
    start = start + 1 == ZONE_DATA ? 0 : start + 1;
  }
  size = size + n > ZONE_DATA ? (u32)ZONE_DATA : size + n;
  head[1] = start;
  head[2] = size;
  if (n) last_byte = text[n - 1];
  __asm__ volatile("dsb sy" ::: "memory");
}

static void put(const char *text, unsigned n) {
  u64 daif = take();
  if (zone) zone_put(text, n);
  else for (unsigned i = 0; i < n && early_len < EARLY_SIZE; ++i) early[early_len++] = text[i];
  give(daif);
}

// A line of its own: after message buffer text that has not ended its line (msgbuf passes cut anywhere), a newline first.
static void line(const char *prefix, const char *text) {
  char buffer[768];
  int n = snprintf(buffer, sizeof buffer, "%s[xnu] %s%s\n", last_byte == '\n' ? "" : "\n", prefix, text ? text : "");
  if (n > 0) put(buffer, (unsigned)n < sizeof buffer ? (unsigned)n : (unsigned)sizeof buffer - 1);
}

// The message buffer's text written since the previous pass (msg_bufx is its next write position); the first pass takes it all.
static void messages(void) {
  static int last = -1;
  struct msgbuf *m = msgbufp;
  if (!m || !m->msg_bufc || m->msg_size <= 0) return;
  const int size = m->msg_size, x = m->msg_bufx;
  if (x < 0 || x >= size) return;
  if (last < 0) last = (x + 1) % size;
  const int n = (x - last + size) % size;
  if (last + n <= size) put(m->msg_bufc + last, (unsigned)n);
  else {
    put(m->msg_bufc + last, (unsigned)(size - last));
    put(m->msg_bufc, (unsigned)(n - (size - last)));
  }
  last = x;
}

// The panic message, once the panic screen has copied it. From the thread (other CPUs running) the text is still being written
// while the screen is drawn, so it is taken when its length has not changed since the previous pass; the panic hook, called once
// the screen is done, takes it at once (`settled`).
static void panic_text(int settled) {
  static int done;
  static unsigned seen;
  volatile const char *source = boot_progress_panic_text;
  if (done || !source[0]) return;
  char text[sizeof boot_progress_panic_text];
  unsigned n = 0;
  for (; n < sizeof text - 1 && source[n]; ++n) text[n] = source[n];
  text[n] = 0;
  if (!settled && n != seen) {seen = n; return;}
  done = 1;
  line("panic: ", text);
}

static u64 ticks(void) {u64 t; __asm__ volatile("isb\n\tmrs %0, cntvct_el0" : "=r"(t) :: "memory"); return t;}
static u64 ticks_per_second(void) {u64 f; __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(f)); return f;}
static void delay_ms(u64 ms) {
  const u64 start = ticks(), wait = ticks_per_second() / 1000 * ms;
  while (ticks() - start < wait) {}
}

// A rough clock of the CPU running this: a counted loop of one subtract and one branch (about one iteration per cycle on these
// cores) timed against the architected counter. Only a magnitude: whether the firmware left the core near its top or far below.
static u32 cpu_mhz_estimate(void) {
  u64 n = 50000000;
  const u64 start = ticks();
  __asm__ volatile("1: subs %0, %0, #1\n\tb.ne 1b" : "+r"(n) :: "cc");
  const u64 elapsed = ticks() - start;
  return elapsed ? (u32)(50000000ULL * ticks_per_second() / elapsed / 1000000) : 0;
}

// The CPU frequency hardware (pipa DT qcom,cpufreq-hw "qcom,cpufreq-hw-epss"): domain 0 at 0x18591000 runs CPUs 0-3 (Silver),
// 1 at 0x18592000 CPUs 4-6 (Gold), 2 at 0x18593000 CPU 7 (Gold Prime). Register facts as Linux's qcom-cpufreq-hw uses them for EPSS
// (epss_soc_data): 0x100 + 4*i the frequency lookup table (bits 31:30 the source, 0 = the 300 MHz alternate clock; bits 7:0 the
// multiple of the 19.2 MHz XO; a repeated frequency ends the table), 0x320 the requested performance level (a table row).
// pipa-cpufreq=1 asks each domain for its top row; the hardware's limits management still caps it. Logged either way.
#define CPUFREQ_DOMAINS 3
#define CPUFREQ_LUT     0x100U
#define CPUFREQ_PERF    0x320U
#define CPUFREQ_ROWS    40U
static u32 cpufreq_mhz(u32 entry) {return ((entry >> 30) & 3) ? (entry & 0xff) * 192 / 10 : 300;}
static void cpufreq(int request) {
  static const u64 phys[CPUFREQ_DOMAINS] = {0x18591000UL, 0x18592000UL, 0x18593000UL};
  static volatile u32 *domain[CPUFREQ_DOMAINS];
  char text[256];
  unsigned at = 0;
  for (unsigned d = 0; d < CPUFREQ_DOMAINS && at + 64 < sizeof text; ++d) {
    if (!domain[d]) domain[d] = (volatile u32 *)ml_io_map(phys[d], 0x1000);
    volatile u32 *r = domain[d];
    if (!r) {at += (unsigned)snprintf(text + at, sizeof text - at, "d%u unmapped | ", d); continue;}
    u32 rows = 0, top = 0, top_mhz = 0, previous = 0;
    for (; rows < CPUFREQ_ROWS; ++rows) {
      u32 mhz = cpufreq_mhz(r[(CPUFREQ_LUT + 4 * rows) / 4]);
      if (rows && mhz == previous) break;
      if (mhz > top_mhz) {top_mhz = mhz; top = rows;}
      previous = mhz;
    }
    if (request && rows) {r[CPUFREQ_PERF / 4] = top; __asm__ volatile("dsb sy" ::: "memory");}
    u32 perf = r[CPUFREQ_PERF / 4];
    at += (unsigned)snprintf(text + at, sizeof text - at, "d%u perf %u (%u MHz) of %u rows, top %u (%u MHz)%s | ", d, perf,
                             perf < rows ? cpufreq_mhz(r[(CPUFREQ_LUT + 4 * perf) / 4]) : 0, rows, top, top_mhz,
                             request ? " requested" : "");
  }
  line("cpufreq: ", text);
}

// One byte of a PMIC peripheral through the SPMI arbiter (RamoopsLogLib's SpmiRead8/SpmiWrite8). 0 = done.
static int spmi_wait(volatile u32 *channel) {
  for (u32 poll = 0; poll < SPMI_POLLS; ++poll) {
    u32 status = channel[SPMI_STATUS / 4];
    if (status & 1) return (status & 0xE) ? 2 : 0;
  }
  return 1;
}
static int spmi_read8(volatile u32 *observer, u8 address, u8 *value) {
  observer[SPMI_CMD / 4] = (1U << 27) | ((u32)address << 4);
  int rc = spmi_wait(observer);
  if (!rc) *value = (u8)observer[SPMI_RDATA0 / 4];
  return rc;
}
static int spmi_update8(volatile u32 *observer, volatile u32 *channel, u8 address, u8 mask, u8 value) {
  u8 old = 0;
  int rc = spmi_read8(observer, address, &old);
  if (rc) return rc;
  channel[SPMI_WDATA0 / 4] = (u8)((old & ~mask) | (value & mask));
  channel[SPMI_CMD / 4] = (u32)address << 4;
  return spmi_wait(channel);
}
// A reset source's type set to warm, as qpnp_config_reset does it: the reset off, at least 10 sleep clock cycles, the type, the
// reset on again when it was on. CTL2 follows CTL.
static int spmi_warm_reset(volatile u32 *observer, volatile u32 *channel, u8 control) {
  u8 enable = 0;
  int rc = spmi_read8(observer, control + 1, &enable);
  if (!rc && (enable & PON_RESET_EN)) {rc = spmi_update8(observer, channel, control + 1, PON_RESET_EN, 0); delay_ms(1);}
  if (!rc) rc = spmi_update8(observer, channel, control, 0x0F, 0x01);
  if (!rc && (enable & PON_RESET_EN)) rc = spmi_update8(observer, channel, control + 1, PON_RESET_EN, PON_RESET_EN);
  return rc;
}

// The PON's reboot reason (bootloader) and the warm type of PS_HOLD and of the power key's and power + volume down's S2 resets
// (RamoopsLogLib's PrepareWarmFastboot, which the firmware already ran; done again so a kernel restart does not depend on it).
// 0 = all written, otherwise RamoopsLogLib's code.
static u32 prepare_warm_fastboot(void) {
  volatile u32 *core = (volatile u32 *)ml_io_map(SPMI_CORE, 0x2000), *cnfg = (volatile u32 *)ml_io_map(SPMI_CNFG, 0x1000);
  if (!core || !cnfg) return 0x01;
  u32 version = core[0], apids = core[1] & 0x7FF;
  int apid = -1;
  if (version < 0x50000000U || version >= 0x70000000U) return 0x10;
  if (!apids || apids > SPMI_MAX_APIDS) return 0x11;
  for (u32 i = 0; i < apids; ++i) {
    u32 map = core[(SPMI_APID_MAP + 4 * i) / 4];
    if (!map || ((map >> 8) & 0xFFF) != PON_PPID) continue;
    if (apid < 0 || (cnfg[(SPMI_CNFG_OWNER + 4 * i) / 4] & 7) == SPMI_EE) apid = (int)i;
  }
  if (apid < 0) return 0x12;
  if ((cnfg[(SPMI_CNFG_OWNER + 4 * (u32)apid) / 4] & 7) != SPMI_EE) return 0x13;
  u64 observer_phys = SPMI_OBSRVR + SPMI_OBS_APID_STRIDE * (u64)apid;
  volatile u8 *observer_page = (volatile u8 *)ml_io_map(observer_phys & ~0xFFFUL, 0x1000);
  volatile u32 *channel = (volatile u32 *)ml_io_map(SPMI_CHNLS + SPMI_CHNL_APID_STRIDE * (u64)apid, 0x1000);
  if (!observer_page || !channel) return 0x02;
  volatile u32 *observer = (volatile u32 *)(observer_page + (observer_phys & 0xFFF));
  u8 type = 0, subtype = 0;
  if (spmi_read8(observer, 0x04, &type) || spmi_read8(observer, 0x05, &subtype)) return 0x20;
  if (type != 0x01 || (subtype != 0x04 && subtype != 0x05)) return 0x21;
  if (spmi_update8(observer, channel, PON_SOFT_RB_SPARE, 0xFE, PON_REASON_BOOTLOADER)) return 0x30;
  if (spmi_warm_reset(observer, channel, PON_PS_HOLD_RST_CTL)) return 0x40;
  if (spmi_warm_reset(observer, channel, PON_KPDPWR_S2_CTL)) return 0x41;
  if (spmi_warm_reset(observer, channel, PON_COMBO_S2_CTL)) return 0x42;
  return 0;
}

static u64 smc3(u64 function, u64 a1, u64 a2, u64 a3) {
  register u64 x0 __asm__("x0") = function, x1 __asm__("x1") = a1, x2 __asm__("x2") = a2, x3 __asm__("x3") = a3;
  __asm__ volatile("smc #0" : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3) :
                   : "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
  return x0;
}

// RamoopsLogLib's RestartIntoFastboot after its PON part (prepare_warm_fastboot, done at boot_progress_start). Interrupts are
// masked; it does not return.
__attribute__((noreturn)) static void restart_into_fastboot(void) {
  __asm__ volatile("msr daifset, #3" ::: "memory");
  if (ps_hold && imem) {
    *imem = RESTART_BOOTLOADER;
    __asm__ volatile("dsb sy" ::: "memory");
    smc3(SCM_WDOG_DEBUG_OFF, 2, 1, 0);
    smc3(SCM_HALT_PMIC_ARB, 1, 0, 0);
    *ps_hold = 0;
    __asm__ volatile("dsb sy" ::: "memory");
    delay_ms(1000);
    static const char fallback[] = "[xnu] restart: PS_HOLD did not restart, PSCI SYSTEM_RESET\n";
    if (zone) zone_put(fallback, sizeof fallback - 1);
  }
  smc3(PSCI_SYSTEM_RESET, 0, 0, 0);
  for (;;) __asm__ volatile("wfe");
}

// The display's counters (LegacyDisplayProbe.cpp, CONFIG_XNU_NATIVE_LEGACY_DISPLAY; weak, absent without it): the probe's
// stage, the frame buffer it took, swaps the clients submitted, CPU copies, retired frames and user client calls.
extern volatile u32 native_legacy_stage __attribute__((weak_import));
extern volatile u32 native_legacy_gop_width __attribute__((weak_import));
extern volatile u32 native_legacy_gop_height __attribute__((weak_import));
extern volatile u32 native_legacy_swap_validation_calls __attribute__((weak_import));
extern volatile u32 native_legacy_transfer_calls __attribute__((weak_import));
extern volatile u32 native_legacy_retirement_calls __attribute__((weak_import));
extern volatile u32 native_legacy_client_calls __attribute__((weak_import));
extern volatile int native_legacy_unit_scale_status __attribute__((weak_import));
extern volatile u32 native_legacy_unit_scale_calls __attribute__((weak_import));
extern volatile u32 native_legacy_chosen_scale __attribute__((weak_import));
extern volatile u32 native_legacy_chosen_rotation __attribute__((weak_import));
// The cores (drivers/ArmCpu): each logical core's start step (7 = running), the cores running, IPIs sent and taken, and the
// SPI block standing in for SGIs.
extern volatile u32 armcpu_smp_step[8] __attribute__((weak_import));
extern volatile u32 armcpu_smp_running __attribute__((weak_import));
extern volatile unsigned long armcpu_ipi_sent __attribute__((weak_import));
extern volatile unsigned long armcpu_ipi_interrupts __attribute__((weak_import));
extern volatile u32 armcpu_spi_ipi_base __attribute__((weak_import));
static void cpu_status(void) {
  if (!&armcpu_smp_step || !&armcpu_smp_running || !&armcpu_ipi_sent || !&armcpu_ipi_interrupts || !&armcpu_spi_ipi_base) return;
  char text[160];
  snprintf(text, sizeof text, "running %u steps %u %u %u %u %u %u %u %u ipi sent %lu taken %lu spi base %u", armcpu_smp_running,
           armcpu_smp_step[0], armcpu_smp_step[1], armcpu_smp_step[2], armcpu_smp_step[3], armcpu_smp_step[4], armcpu_smp_step[5],
           armcpu_smp_step[6], armcpu_smp_step[7], armcpu_ipi_sent, armcpu_ipi_interrupts, armcpu_spi_ipi_base);
  line("cpus: ", text);
}

// The touch controller (TouchProbe.cpp, pipa-touch=<stage>): its status rows (stages, clocks and pins, chip id and firmware info,
// transfer errors), the trace rows that matter once it runs (reports, download, SPI modes), and every finished stroke
// {start ms, ms, x0, y0, x1, y1, reports} with the extent all contacts reached (HID units), for the axis check.
extern volatile unsigned native_touch_stage_req __attribute__((weak_import));
extern char native_touch_rows[4][120] __attribute__((weak_import));
extern char native_touch_trace[8][120] __attribute__((weak_import));
extern volatile int native_touch_strokes[16][7] __attribute__((weak_import));
extern volatile unsigned native_touch_stroke_count __attribute__((weak_import));
extern volatile int native_touch_extent[4] __attribute__((weak_import));
static void touch_row(const char *prefix, const char *row, unsigned cap) {
  char text[128];
  unsigned n = 0;
  for (; n < cap && n < sizeof text - 1 && row[n]; ++n) text[n] = row[n];
  text[n] = 0;
  if (n) line(prefix, text);
}
static void touch_status(void) {
  if (!&native_touch_stage_req || !&native_touch_rows || !&native_touch_trace || !native_touch_stage_req) return;
  for (unsigned i = 0; i < 4; ++i) touch_row("touch: ", native_touch_rows[i], sizeof native_touch_rows[i]);
  static const unsigned traces[] = {3, 4, 7, 0, 2};
  for (unsigned i = 0; i < sizeof traces / sizeof traces[0]; ++i)
    touch_row("touch: ", native_touch_trace[traces[i]], sizeof native_touch_trace[traces[i]]);
}
static void touch_strokes(void) {
  static unsigned logged;
  if (!&native_touch_strokes || !&native_touch_stroke_count || !&native_touch_extent) return;
  unsigned count = native_touch_stroke_count;
  if (count - logged > 16) logged = count - 16;
  for (; logged < count; ++logged) {
    volatile int *r = native_touch_strokes[logged & 15];
    char text[160];
    snprintf(text, sizeof text, "#%u at %d ms for %d ms (%d,%d) -> (%d,%d) %d reports, extent x %d..%d y %d..%d", logged, r[0], r[1],
             r[2], r[3], r[4], r[5], r[6], native_touch_extent[0], native_touch_extent[1], native_touch_extent[2],
             native_touch_extent[3]);
    line("touch stroke: ", text);
  }
}
// The keys (Buttons.c, pipa-buttons=1): its row after every change of the pressed set (at most one per pass, the latest).
extern char native_buttons_row[120] __attribute__((weak_import));
extern volatile unsigned native_buttons_changes __attribute__((weak_import));
extern volatile int native_buttons_error __attribute__((weak_import));
static void buttons_status(void) {
  static unsigned logged, error_logged;
  if (!&native_buttons_row || !&native_buttons_changes || !&native_buttons_error) return;
  if (native_buttons_error && !error_logged) {
    char text[32];
    snprintf(text, sizeof text, "stopped, error %d", native_buttons_error);
    line("buttons: ", text);
    error_logged = 1;
  }
  unsigned changes = native_buttons_changes;
  if (changes == logged) return;
  logged = changes;
  touch_row("buttons: ", native_buttons_row, sizeof native_buttons_row);
}
// The GPU bring-up (GpuProbe.cpp, pipa-gpu=<mode>): its rows whenever they change, and its step every 10 s while it runs (a hang
// in a register read leaves the step it was at).
extern char native_gpu_rows[17][120] __attribute__((weak_import));
extern volatile unsigned native_gpu_changes __attribute__((weak_import));
extern volatile unsigned native_gpu_step __attribute__((weak_import));
static void gpu_status(int periodic) {
  static unsigned logged;
  if (!&native_gpu_rows || !&native_gpu_changes || !&native_gpu_step || !native_gpu_step) return;
  if (periodic && native_gpu_step != 99) {
    char text[32];
    snprintf(text, sizeof text, "step %u", native_gpu_step);
    line("gpu: ", text);
  }
  unsigned changes = native_gpu_changes;
  if (changes == logged) return;
  logged = changes;
  for (unsigned i = 0; i < 17; ++i) touch_row("gpu: ", native_gpu_rows[i], sizeof native_gpu_rows[i]);
}

// The lines sheng's progress page shows for other components (KeyStoreProbe.cpp writes line 0); logged with the display line.
char boot_progress_ext[8][48];

static void display_status(void) {
  if (!&native_legacy_stage || !&native_legacy_gop_width || !&native_legacy_gop_height || !&native_legacy_swap_validation_calls ||
      !&native_legacy_transfer_calls || !&native_legacy_retirement_calls || !&native_legacy_client_calls) return;
  char text[224];
  snprintf(text, sizeof text, "stage %u gop %ux%u swaps %u copies %u retired %u client calls %u unit scale %d/%u chosen %u rotation %u",
           native_legacy_stage, native_legacy_gop_width, native_legacy_gop_height, native_legacy_swap_validation_calls,
           native_legacy_transfer_calls, native_legacy_retirement_calls, native_legacy_client_calls,
           &native_legacy_unit_scale_status ? native_legacy_unit_scale_status : -9,
           &native_legacy_unit_scale_calls ? native_legacy_unit_scale_calls : 0, &native_legacy_chosen_scale ? native_legacy_chosen_scale : 0,
           &native_legacy_chosen_rotation ? native_legacy_chosen_rotation : 0);
  line("display: ", text);
  for (unsigned i = 0; i < 8; ++i) {
    if (!boot_progress_ext[i][0]) continue;
    char ext[sizeof boot_progress_ext[i] + 1];
    for (unsigned c = 0; c < sizeof boot_progress_ext[i]; ++c) ext[c] = boot_progress_ext[i][c];
    ext[sizeof boot_progress_ext[i]] = 0;
    line("ext: ", ext);
  }
}

// The display hand-off of sheng's progress page (LegacyDisplayProbe.cpp asks before a CPU scanout copy): there is no page on
// this board, so the display always takes the frame buffer; the first copy is logged as the first frame.
int boot_progress_prepare_scanout(void) {
  static int first = 1;
  if (first) {first = 0; line("display: ", "first scanout copy, the display takes the frame buffer");}
  return 1;
}
void boot_progress_cancel_scanout(void) {}
// The page's anchor on the display's frame buffer mapping (LegacyDisplayProbe.cpp offers it once): no page, so none is taken.
int boot_progress_pulse_anchor(unsigned long base, u64 bytes, unsigned width, unsigned height, unsigned rowbytes) {
  (void)base; (void)bytes; (void)width; (void)height; (void)rowbytes;
  return 0;
}

static void writer(void *argument, int result) {
  (void)argument; (void)result;
  const u64 start = ticks(), limit = ticks_per_second() * restart_after;
  for (u32 pass = 1;; ++pass) {
    messages();
    panic_text(0);
    if (pass % 100 == 0) {display_status(); cpu_status(); touch_status();}
    if (pass % 5 == 0) touch_strokes();
    buttons_status();
    gpu_status(pass % 100 == 0);
    if (restart_after && ticks() - start >= limit) {
      char text[64];
      snprintf(text, sizeof text, "%u s after the zone was mapped (pipa-restart-after)", restart_after);
      line("restart into fastboot: ", text);
      restart_into_fastboot();
    }
    if (restart_crashes && ui_crashes >= restart_crashes) {
      messages();
      char text[80];
      snprintf(text, sizeof text, "%u SpringBoard/backboardd crashes (pipa-restart-crashes)", ui_crashes);
      line("restart into fastboot: ", text);
      restart_into_fastboot();
    }
    IOSleep(PASS_MS);
  }
}

// Called by the panic screen (scripts/xnu-panic-screen.c, PANIC_HOOK) once it has drawn the first panic, interrupts masked, on
// the only running CPU: the message buffer's rest and the panic text into the zone, then the restart after
// pipa-panic-restart-after seconds (0 = stay on the panic screen). The lock cannot be held by a preempted thread (take masks
// interrupts), but a panic inside a locked section would leave it set: it is released after a bounded wait.
void boot_progress_panic_hook(void) {
  for (u32 spin = 0; spin < 1000000 && __atomic_load_n(&lock, __ATOMIC_ACQUIRE); ++spin) {}
  __atomic_store_n(&lock, 0, __ATOMIC_RELEASE);
  if (!zone) return;
  messages();
  panic_text(1);
  if (!panic_restart_after) return;
  char text[64];
  snprintf(text, sizeof text, "in %u s (pipa-panic-restart-after)", panic_restart_after);
  line("panic: restart into fastboot ", text);
  delay_ms((u64)panic_restart_after * 1000);
  restart_into_fastboot();
}

void boot_progress_init(void) {line("", "boot_progress_init (stages kept until the ramoops zone is mapped)");}

void boot_progress_start(void) {
  volatile u8 *mapped = (volatile u8 *)ml_io_map(ZONE_BASE, ZONE_SIZE);
  if (!mapped) return;
  u64 daif = take();
  zone = mapped;
  static const char banner[] = "\n[xnu] ---- kernel: ramoops console zone mapped; the stages before it follow ----\n";
  zone_put(banner, sizeof banner - 1);
  zone_put(early, early_len);
  give(daif);
  PE_parse_boot_argn("pipa-restart-after", &restart_after, sizeof restart_after);
  PE_parse_boot_argn("pipa-panic-restart-after", &panic_restart_after, sizeof panic_restart_after);
  PE_parse_boot_argn("pipa-restart-crashes", &restart_crashes, sizeof restart_crashes);
  u32 pon = prepare_warm_fastboot();
  ps_hold = (volatile u32 *)ml_io_map(PS_HOLD, 0x1000);
  volatile u8 *imem_page = (volatile u8 *)ml_io_map(IMEM_RESTART_REASON & ~0xFFFUL, 0x1000);
  imem = imem_page ? (volatile u32 *)(imem_page + (IMEM_RESTART_REASON & 0xFFF)) : 0;
  char text[128];
  snprintf(text, sizeof text, "PON 0x%x (%s), PS_HOLD %s, after %u s, after a panic %u s", pon, pon ? "not prepared" : "warm, bootloader",
           ps_hold && imem ? "mapped" : "NOT mapped", restart_after, panic_restart_after);
  line("restart into fastboot: ", text);
  snprintf(text, sizeof text, "about %u MHz (a counted loop against the counter)", cpu_mhz_estimate());
  line("boot cpu: ", text);
  u32 want_cpufreq = 0;
  PE_parse_boot_argn("pipa-cpufreq", &want_cpufreq, sizeof want_cpufreq);
  cpufreq(want_cpufreq == 1);
  if (want_cpufreq == 1) {
    snprintf(text, sizeof text, "about %u MHz after the request", cpu_mhz_estimate());
    line("boot cpu: ", text);
  }
  void *thread = 0;
  if (kernel_thread_start(writer, 0, &thread) == 0) thread_deallocate(thread);
  else line("", "message buffer thread NOT started");
}

int boot_progress_active(void) {return 0;}
void boot_progress_bind(unsigned long base, unsigned width, unsigned height, unsigned rowbytes, unsigned depth) {
  (void)base; (void)width; (void)height; (void)rowbytes; (void)depth;
}
void boot_progress_putc(char c) {(void)c;}
void boot_progress_stage(const char *text) {line("stage: ", text);}
void boot_progress_irq(u64 pc, u64 cpsr, u64 sp) {(void)pc; (void)cpsr; (void)sp;}
void boot_progress_poll(void) {}

static int same(const char *a, const char *b) {
  while (*a && *a == *b) {++a; ++b;}
  return *a == *b;
}

// boot_progress_user_breakpoint, boot_progress_user_abort and boot_progress_critical_exit are BootFaultProbe.c's (the EL0
// exception records); a crashing exit below asks it for the crash line, as sheng's progress page does.
extern void boot_progress_exit_fault(int pid, unsigned signal, char *out, unsigned long capacity);
extern void boot_progress_exit_reason(int pid, unsigned signal, char *out, unsigned long capacity);

void boot_progress_process_stage(int pid, const char *name, const char *stage) {
  char prefix[96];
  snprintf(prefix, sizeof prefix, "%d %s: ", pid, name ? name : "?");
  line(prefix, stage);
}

// Every exit; for one by signal 3..12 (not 9) also the crash line (pid, name, signal, PC, LR, ESR, FAR, the shared cache slide
// and a frame walk), and for SIGABRT the uncaught exception's reason found in the process's memory. At most 48 crash lines.
void boot_progress_exit(int pid, const char *name, int status, unsigned reason_namespace, u64 reason_code) {
  char text[640];
  snprintf(text, sizeof text, "%d %s status 0x%x reason %u/0x%llx", pid, name ? name : "?", status, reason_namespace,
           (unsigned long long)reason_code);
  line("exit ", text);
  const unsigned signal = (unsigned)status & 0x7f;
  if (signal < 3 || signal > 12 || signal == 9) return;
  if (name && (same(name, "SpringBoard") || same(name, "backboardd"))) ui_crashes = ui_crashes + 1;
  if (user_faults >= 48) return;
  ++user_faults;
  text[0] = 0;
  boot_progress_exit_fault(pid, signal, text, sizeof text);
  if (text[0]) line("crash: ", text);
  if (signal != 6) return;
  text[0] = 0;
  boot_progress_exit_reason(pid, signal, text, sizeof text);
  if (text[0]) line("crash reason: ", text);
}
