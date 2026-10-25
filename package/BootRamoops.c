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
// The PON block's PMIC watchdog (mainline pm8916_wdt.c): S1 counts down from the last pet and barks, S2 then resets the PMIC
// (and with it the SoC through PS_HOLD) after its own delay. S2_CTL holds the reset type (0x01 warm, 0x04 shutdown, 0x07 hard),
// S2_CTL2 bit 7 arms the stage-2 reset, PET bit 0 pets. PON_INT_RT_STS bit 6 is the bark, POFF_REASON1 bit 2 says a PMIC
// watchdog caused the last power off.
#define PON_WD_S1_TIMER       0x54
#define PON_WD_S2_TIMER       0x55
#define PON_WD_S2_CTL         0x56
#define PON_WD_S2_CTL2        0x57
#define PON_WD_PET            0x58
#define PON_WD_S2_EN          0x80
#define PON_WD_PET_BIT        0x01
#define PON_WD_RESET_WARM     0x01
#define PON_RT_STS            0x10
#define PON_POFF_REASON1      0x0C
#define PON_POFF_REASON2      0x0D
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
extern int strncmp(const char *, const char *, unsigned long);
extern unsigned long strlcpy(char *, const char *, unsigned long);
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
static volatile u32 *ponObserver, *ponChannel;   // the PON's observer and channel windows, resolved by prepare_warm_fastboot
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
  ponObserver = observer;   // the watchdog thread uses the same two windows
  ponChannel = channel;
  u8 type = 0, subtype = 0;
  if (spmi_read8(observer, 0x04, &type) || spmi_read8(observer, 0x05, &subtype)) return 0x20;
  if (type != 0x01 || (subtype != 0x04 && subtype != 0x05)) return 0x21;
  if (spmi_update8(observer, channel, PON_SOFT_RB_SPARE, 0xFE, PON_REASON_BOOTLOADER)) return 0x30;
  if (spmi_warm_reset(observer, channel, PON_PS_HOLD_RST_CTL)) return 0x40;
  if (spmi_warm_reset(observer, channel, PON_KPDPWR_S2_CTL)) return 0x41;
  if (spmi_warm_reset(observer, channel, PON_COMBO_S2_CTL)) return 0x42;
  return 0;
}

// ---- The PON's own watchdog: the last resort when the kernel stops running.
// A wedged interconnect or a dead CPU leaves nothing inside the kernel able to restart the tablet, but the PMIC keeps its own
// timer over SPMI (a separate two-wire bus) and pulls PS_HOLD itself. 60 s without a pet is the bite; a thread pets it every
// 2 s. The warm reset type and the bootloader's fastboot reason (SOFT_RB_SPARE) are already written by prepare_warm_fastboot,
// and a warm reset keeps DRAM, so the next boot comes up in fastboot with this ring intact.
// `pipa-wd=0` turns it off (14 bytes on the command line); `pipa-wd=2` or `pipa-wd=test` arms it and stops feeding after 15 s,
// which is how one round can watch the reset actually happen. Default (no argument) is armed and fed.
#define WD_BITE_SECONDS 60
#define WD_FEED_MS      2000
#define WD_TEST_MS      15000
#define WD_LOG_EVERY    150                     // one "feed N ok" every 5 minutes, not every 2 s

static int spmi_write8(volatile u32 *channel, u8 address, u8 value) {
  if (!channel) return 1;
  channel[SPMI_WDATA0 / 4] = value;
  channel[SPMI_CMD / 4] = (u32)address << 4;
  return spmi_wait(channel);
}
static int wd_read(u8 address, u8 *value) {
  return !ponObserver || spmi_read8(ponObserver, address, value);
}
// Write a masked byte, then read the register back: 0 only when the peripheral holds what was written.
static int wd_write_checked(u8 address, u8 mask, u8 value) {
  if (!ponObserver || !ponChannel) return 1;
  if (spmi_update8(ponObserver, ponChannel, address, mask, value)) return 2;
  u8 readback = 0;
  if (spmi_read8(ponObserver, address, &readback)) return 3;
  return (readback & mask) == (value & mask) ? 0 : 4;
}
// 0 = armed; otherwise the step that failed, and the log carries the code.
static u32 wd_arm(void) {
  if (wd_write_checked(PON_WD_S1_TIMER, 0xFF, WD_BITE_SECONDS)) return 0x10;
  if (wd_write_checked(PON_WD_S2_TIMER, 0xFF, 0)) return 0x11;
  if (wd_write_checked(PON_WD_S2_CTL, 0x0F, PON_WD_RESET_WARM)) return 0x12;
  if (wd_write_checked(PON_WD_S2_CTL2, PON_WD_S2_EN, PON_WD_S2_EN)) return 0x13;
  return 0;
}
// The pet is write-only in practice (it reads back as a self-clearing bit), so the SPMI transaction's own status is the health
// signal here; the S2 enable bit is re-read every 30 s below.
static int wd_feed(void) {return spmi_write8(ponChannel, PON_WD_PET, PON_WD_PET_BIT);}
// On a failed arm, put back what was there: the bootloader may have left a watchdog armed, and taking it away without arming
// ours would leave the tablet with none.
static void wd_restore(u8 s1, u8 s2, u8 type, u8 cntl) {
  wd_write_checked(PON_WD_S1_TIMER, 0xFF, s1);
  wd_write_checked(PON_WD_S2_TIMER, 0xFF, s2);
  wd_write_checked(PON_WD_S2_CTL, 0x0F, type & 0x0F);
  if (cntl & PON_WD_S2_EN) wd_write_checked(PON_WD_S2_CTL2, PON_WD_S2_EN, PON_WD_S2_EN);
}

static void watchdog_thread(void *argument, int result) {
  (void)result;
  const unsigned mode = (unsigned)(unsigned long)argument;   // 1 = fed, 2 = fed for 15 s then left to bite
  char text[96];
  u8 s1 = 0, s2 = 0, type = 0, cntl = 0, sts = 0, spare = 0, poff1 = 0, poff2 = 0;
  if (wd_read(PON_WD_S1_TIMER, &s1) || wd_read(PON_WD_S2_TIMER, &s2) || wd_read(PON_WD_S2_CTL, &type) ||
      wd_read(PON_WD_S2_CTL2, &cntl) || wd_read(PON_RT_STS, &sts) || wd_read(PON_SOFT_RB_SPARE, &spare) ||
      wd_read(PON_POFF_REASON1, &poff1) || wd_read(PON_POFF_REASON2, &poff2)) {
    line("wd: ", "not armed (the PON does not answer over SPMI)");
    return;
  }
  snprintf(text, sizeof text, "pon s1 %02x s2 %02x type %02x cntl %02x sts %02x", s1, s2, type, cntl, sts);
  line("wd: ", text);
  snprintf(text, sizeof text, "pon spare %02x poff %02x/%02x, armed by %s", spare, poff1, poff2,
           (cntl & PON_WD_S2_EN) ? "the bootloader" : "nobody");
  line("wd: ", text);
  // Whoever had it armed, it is ours now: stop it while the timers change, then arm with our values. Nothing can bite in
  // between, because the stage-2 reset stays off until the last write.
  if ((cntl & PON_WD_S2_EN) && wd_write_checked(PON_WD_S2_CTL2, PON_WD_S2_EN, 0)) {
    line("wd: ", "not armed (the bootloader's timer does not let go)");
    return;
  }
  const u32 failure = wd_arm();
  if (failure) {
    snprintf(text, sizeof text, "not armed (arm step 0x%02x)", (unsigned)failure);
    line("wd: ", text);
    wd_restore(s1, s2, type, cntl);
    return;
  }
  // One pet before the thread settles in; if the write cannot be made, take the arming back rather than leave a timer that
  // nothing is feeding.
  int pet = wd_feed();
  for (unsigned retry = 0; pet && retry < 5; ++retry) {
    IOSleep(200);
    pet = wd_feed();
  }
  if (pet) {
    wd_write_checked(PON_WD_S2_CTL2, PON_WD_S2_EN, 0);
    line("wd: ", "not armed (the pet does not go through; disarmed)");
    return;
  }
  snprintf(text, sizeof text, "armed %u s, feeding every %u s%s", WD_BITE_SECONDS, WD_FEED_MS / 1000,
           mode == 2 ? ", test mode" : "");
  line("wd: ", text);
  unsigned feeds = 0, failing = 0;
  for (;;) {
    IOSleep(WD_FEED_MS);
    if (mode == 2 && (feeds + 1) * WD_FEED_MS >= WD_TEST_MS) {
      line("wd: ", "test mode, stopping the feed at 15 s");
      return;   // no more pets: the PMIC resets the tablet 60 s after the last one
    }
    ++feeds;
    const int rc = wd_feed();
    if (rc) {
      if (!failing) {snprintf(text, sizeof text, "feed %u failed (spmi %d), retrying", feeds, rc); line("wd: ", text);}
      if (++failing == 10) line("wd: ", "10 feeds failed; the PMIC will reset the tablet");
      continue;
    }
    if (failing) {
      snprintf(text, sizeof text, "feed %u ok again after %u failures", feeds, failing);
      line("wd: ", text);
      failing = 0;
    } else if (feeds == 1 || feeds % WD_LOG_EVERY == 0) {
      snprintf(text, sizeof text, "feed %u ok", feeds);
      line("wd: ", text);
    }
    if (feeds % 15 == 0) {   // still armed? (the bootloader, a reset of the PMIC block, or someone else may have cleared it)
      u8 now = 0;
      if (!spmi_read8(ponObserver, PON_WD_S2_CTL2, &now) && !(now & PON_WD_S2_EN)) {
        line("wd: ", "found it disarmed, arming again");
        if (wd_arm()) line("wd: ", "re-arm failed");
      }
    }
  }
}
// Started from boot_progress_start, before anything slow: a boot that takes ten minutes must not look like a hang, and the
// feeder has to be running before the first 60 s can expire.
static void watchdog_start(void) {
  unsigned mode = 1;
  char argument[16] = {0};
  if (PE_parse_boot_argn("pipa-wd", argument, sizeof argument)) {
    const unsigned numeric = (unsigned)(u8)argument[0] | ((unsigned)(u8)argument[1] << 8) |
                             ((unsigned)(u8)argument[2] << 16) | ((unsigned)(u8)argument[3] << 24);
    if (!numeric) mode = 0;
    else if (numeric == 2 || !__builtin_strncmp(argument, "test", 4)) mode = 2;
  }
  if (!mode) {line("wd: ", "off (pipa-wd=0)"); return;}
  void *thread = 0;
  if (kernel_thread_start(watchdog_thread, (void *)(unsigned long)mode, &thread) != 0) {
    line("wd: ", "not armed (no thread)");
    return;
  }
  thread_deallocate(thread);
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
extern unsigned native_legacy_swap_pacing(char *out, unsigned cap) __attribute__((weak_import));
// The scanout hand-off's retry record and the refresh tick's state as two short lines, and the tick's watchdog (NativeLegacyDisplay.cpp
// native_legacy_vsync_kick: arms a tick again that stopped while a client waits; cheap while ticks arrive).
extern unsigned native_legacy_prepare_brief(char *out, unsigned cap) __attribute__((weak_import));
extern unsigned native_legacy_vsync_brief(char *out, unsigned cap) __attribute__((weak_import));
extern int native_legacy_vsync_kick(void) __attribute__((weak_import));
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
// The battery (NativeBattery.cpp, pipa-battery=<0|1|2>): its row after every change of the reading (at most one per pass),
// and the state at which the driver stopped when it could not publish (the object itself is an IOPMPowerSource in the registry).
extern char native_battery_row[120] __attribute__((weak_import));
extern volatile unsigned native_battery_changes __attribute__((weak_import));
extern volatile int native_battery_error __attribute__((weak_import));
extern volatile unsigned native_battery_published __attribute__((weak_import));
static void battery_status(void) {
  static unsigned logged, error_logged, published_logged;
  if (!&native_battery_row || !&native_battery_changes || !&native_battery_error || !&native_battery_published) return;
  if (native_battery_published && !published_logged) {line("battery: ", "IOPMPowerSource published"); published_logged = 1;}
  if (native_battery_error && !error_logged) {
    char text[48];
    snprintf(text, sizeof text, "%s, error %d", native_battery_published ? "read failed" : "stopped", native_battery_error);
    line("battery: ", text);
    error_logged = 1;
  }
  unsigned changes = native_battery_changes;
  if (changes == logged) return;
  logged = changes;
  touch_row("battery: ", native_battery_row, sizeof native_battery_row);
}
// The scripted touches (Gestures.c, pipa-gesture=<seconds>): one row per step in a ring of eight, each printed once, and the
// error at which the script gave up. A step's row carries the HID reports it caused (the event reached the event driver) and the
// presented frames (the UI reacted).
extern char native_gesture_rows[8][120] __attribute__((weak_import));
extern volatile unsigned native_gesture_written __attribute__((weak_import));
extern volatile int native_gesture_error __attribute__((weak_import));
extern char native_gesture_timeline[4096] __attribute__((weak_import));
extern volatile unsigned native_gesture_timeline_seq __attribute__((weak_import));
static void gesture_status(void) {
  static unsigned logged, error_logged;
  if (!&native_gesture_rows || !&native_gesture_written || !&native_gesture_error) return;
  if (native_gesture_error && !error_logged) {
    char text[48];
    snprintf(text, sizeof text, "error %d", native_gesture_error);
    line("gesture: ", text);
    error_logged = 1;
  }
  unsigned written = native_gesture_written;
  if (written - logged > 8) logged = written - 8;
  for (; logged < written; ++logged) touch_row("gesture: ", native_gesture_rows[logged & 7], sizeof native_gesture_rows[0]);
  // The swap timeline the script kept at the end of an animation (Gestures.c anim_timeline), once per copy.
  static unsigned timeline_logged;
  if (&native_gesture_timeline && &native_gesture_timeline_seq && native_gesture_timeline_seq != timeline_logged) {
    timeline_logged = native_gesture_timeline_seq;
    char text[128];
    for (const char *at = native_gesture_timeline; *at;) {
      const char *end = at;
      while (*end && *end != '\n') ++end;
      unsigned n = (unsigned)(end - at) < sizeof text - 1 ? (unsigned)(end - at) : sizeof text - 1;
      for (unsigned i = 0; i < n; ++i) text[i] = at[i];
      text[n] = 0;
      line("swaps: ", text);
      at = *end ? end + 1 : end;
    }
  }
}
// The DDR's frequency residency, from the AOP's DDR statistics in its message RAM (downstream DTS qcom,ddr-stats@c3f0000: the table at
// 0xC300000 plus the offset word at 0xC3F001C, 0xF00B8 on the tablet; msm-4.19 soc_sleep_stats.c): a magic word, the entry count, then per entry a name word
// (bits 15:8 1 for a frequency, its MHz in bits 31:16), a count and a 64-bit time in 19.2 MHz ticks. Every 10 s the frequencies whose
// time grew since the last row, with the milliseconds they took: which DDR level the GPU and the CPUs actually ran on (2026-10-25: the
// GPU's copies moved about 16 GB/s, half of what the LPDDR5 at 2736 MHz carries, and the GMU's bandwidth vote changed nothing).
#define AOP_MSG_RAM 0xC300000UL
#define DDR_STATS_OFFSET_WORD 0xC3F001CUL
#define DDR_STATS_MAGIC 0xA1157A75U
static void ddr_status(void) {
  static volatile u32 *stats, *offset_page;
  static u64 last[32];
  static u32 mhz[32];
  static int failed;
  if (failed) return;
  static u32 offset;
  if (!stats) {
    offset_page = (volatile u32 *)ml_io_map(DDR_STATS_OFFSET_WORD & ~0xFFFUL, 0x1000);
    if (!offset_page) {failed = 1; line("ddr: ", "no mapping"); return;}
    offset = offset_page[(DDR_STATS_OFFSET_WORD & 0xFFFUL) / 4];
    // The table and its 32 entries in one mapping: two pages from the table's own.
    const u64 table = AOP_MSG_RAM + offset;
    if ((offset & 3) || offset >= 0x100000) {
      char text[64];
      snprintf(text, sizeof text, "stats offset %x outside the message RAM", offset);
      line("ddr: ", text); failed = 1; return;
    }
    volatile u8 *pages = (volatile u8 *)ml_io_map(table & ~0xFFFUL, 0x2000);
    if (!pages) {failed = 1; line("ddr: ", "no table mapping"); return;}
    stats = (volatile u32 *)(pages + (table & 0xFFFUL));
  }
  const u32 magic = stats[0], count = stats[1];
  if (magic != DDR_STATS_MAGIC || !count || count > 32 || ((AOP_MSG_RAM + offset) & 0xFFFUL) + 8 + count * 16 > 0x2000) {
    char text[80];
    snprintf(text, sizeof text, "no DDR statistics at %x: magic %08x count %u", offset, magic, count);
    line("ddr: ", text); failed = 1; return;
  }
  // The raw table at the first row and every 60 s after it: whether the AOP keeps it current without being asked.
  static unsigned calls;
  if (calls++ % 6 == 0) {
    char raw[220];
    unsigned at = (unsigned)snprintf(raw, sizeof raw, "table at %x: %u entries", offset, count);
    for (u32 i = 0; i < count && at + 30 < sizeof raw; ++i) {
      volatile u32 *entry = stats + 2 + i * 4;
      at += (unsigned)snprintf(raw + at, sizeof raw - at, " %x/%u/%llu", entry[0], entry[1],
                               (unsigned long long)(((u64)entry[2] | ((u64)entry[3] << 32)) / 19200));
      if (at > 160) {line("ddr: ", raw); at = (unsigned)snprintf(raw, sizeof raw, "...");}
    }
    line("ddr: ", raw);
  }
  char text[200];
  unsigned at = (unsigned)snprintf(text, sizeof text, "MHz:ms");
  u64 total = 0;
  for (u32 i = 0; i < count && at + 16 < sizeof text; ++i) {
    volatile u32 *entry = stats + 2 + i * 4;
    const u32 name = entry[0];
    const u64 ticks = (u64)entry[2] | ((u64)entry[3] << 32);
    if (((name >> 8) & 0xFF) != 1) continue;   // a low-power mode, not a frequency
    const u64 grew = last[i] && ticks >= last[i] ? ticks - last[i] : 0;
    last[i] = ticks;
    mhz[i] = name >> 16;
    total += grew;
    if (grew >= 19200) at += (unsigned)snprintf(text + at, sizeof text - at, " %u:%llu", mhz[i], grew / 19200);
  }
  if (total) line("ddr: ", text);
}

// The GPU bring-up (GpuProbe.cpp, pipa-gpu=<mode>): each row whose text changed, and its step every 10 s while it runs (a hang
// in a register read leaves the step it was at). Once the device is published its live row changes every few seconds; only that
// row is written then.
extern char native_gpu_rows[31][120] __attribute__((weak_import));
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
  static char last[31][120];
  for (unsigned i = 0; i < 31; ++i) {
    if (!strncmp(last[i], native_gpu_rows[i], sizeof last[i])) continue;
    strlcpy(last[i], native_gpu_rows[i], sizeof last[i]);
    touch_row("gpu: ", last[i], sizeof last[i]);
  }
}

// The zero-copy plan's P0 map (MdssProbe.cpp): the apps SMMU's display streams, the frame buffer and the compositor's
// surfaces, with no display controller register read (an SError from that block panicked the kernel; the file says why).
// Each row whose text changed, and its step every 10 s while it starts, so a read that hangs still leaves the step it was at.
extern char native_mdss_rows[31][120] __attribute__((weak_import));
extern volatile unsigned native_mdss_changes __attribute__((weak_import));
extern volatile unsigned native_mdss_step __attribute__((weak_import));
static void mdss_status(int periodic) {
  static unsigned logged;
  if (!&native_mdss_rows || !&native_mdss_changes || !&native_mdss_step || !native_mdss_step) return;
  if (periodic && native_mdss_step != 99) {
    char text[32];
    snprintf(text, sizeof text, "step %u", native_mdss_step);
    line("mdss: ", text);
  }
  unsigned changes = native_mdss_changes;
  if (changes == logged) return;
  logged = changes;
  static char last[31][120];
  for (unsigned i = 0; i < 31; ++i) {
    if (!native_mdss_rows[i][0]) continue;
    if (!strncmp(last[i], native_mdss_rows[i], sizeof last[i])) continue;
    strlcpy(last[i], native_mdss_rows[i], sizeof last[i]);
    touch_row("mdss: ", last[i], sizeof last[i]);
  }
}

// P1 of the same plan (FlipProbe.cpp): the two planes' address and stride written back unchanged (P1a), red and green in our
// own 7424-stride buffers and back (P1b), and the SMMU gate that gives the display's stream our own translation with an
// identity map of the frame buffer and enables it (P1c). One row per event, one progress row for the gate's steps, plus the
// watchdog if it had to step in.
extern char native_flip_rows[16][120] __attribute__((weak_import));
extern volatile unsigned native_flip_changes __attribute__((weak_import));
static void flip_status(void) {
  static unsigned logged;
  if (!&native_flip_rows || !&native_flip_changes) return;
  unsigned changes = native_flip_changes;
  if (changes == logged) return;
  logged = changes;
  static char last[16][120];
  for (unsigned i = 0; i < 16; ++i) {
    if (!native_flip_rows[i][0]) continue;
    if (!strncmp(last[i], native_flip_rows[i], sizeof last[i])) continue;
    strlcpy(last[i], native_flip_rows[i], sizeof last[i]);
    touch_row("flip: ", last[i], sizeof last[i]);
  }
}

// The Metal stack above the GPU, as UniXNU-tb128fu's BootLog.c reports it: the GPU addon written into Data and its overlay mounts,
// the plugin's opens and progress markers with its two text lines, the GPU clients, the newest opens of the accelerator, the
// scanout client's waits for a frame, the fence waits, and the display's GPU hand-off fallbacks. Each row is written when its text
// changes (every 10 s and before a restart).
struct native_adreno_client_record {char name[20]; u32 opens, refused, closes, slot, peak_mib, last_refusal;};
extern struct native_adreno_client_record native_adreno_client_log[8] __attribute__((weak_import));
struct native_adreno_metal_diag_view {
  u32 opens, busy, bad, closes, reports, property_reads, foreign_opens, foreign_type, name_reads, calls, last_selector, last_status,
      ext_entries, ext_selector, ext_status, ext_in, ext_out, tunnel_calls, probes, sel_bits[8], mb_maps, mb_calls, marks_total, marks[16],
      chk_macf, chk_res, chk_type, type_pass_lo, type_pass_hi, attach_status, inflight, inflight_since_calls, forced_unmaps;
  char opener[3][17], reader[3][17], name_reader[17], starter[3][17], report[64], report2[64];
};
extern struct native_adreno_metal_diag_view native_adreno_metal_diag __attribute__((weak_import));
struct native_adreno_open_record {char name[20]; u32 type, macf, res, attach, foreign;};
extern struct native_adreno_open_record native_adreno_open_log[8] __attribute__((weak_import));
extern volatile u32 native_adreno_open_log_count __attribute__((weak_import));
// Processes that read MetalPluginName (NativeAdrenoPublish.cpp native_adreno_metal_askers), in the order they first asked.
struct native_adreno_asker_record {char name[20]; u32 reads;};
extern struct native_adreno_asker_record native_adreno_metal_askers[12] __attribute__((weak_import));
extern volatile int native_gpu_addon_files __attribute__((weak_import)), native_gpu_addon_written __attribute__((weak_import)),
    native_gpu_addon_same __attribute__((weak_import)), native_gpu_addon_failed __attribute__((weak_import)),
    native_gpu_addon_error __attribute__((weak_import));
extern volatile int native_gpu_addon_stage __attribute__((weak_import)), native_gpu_addon_status __attribute__((weak_import)),
    native_gpu_addon2_stage __attribute__((weak_import)), native_gpu_addon2_status __attribute__((weak_import)),
    native_gpu_addon3_stage __attribute__((weak_import)), native_gpu_addon3_status __attribute__((weak_import));
extern volatile u64 native_gop_wait_stats[7] __attribute__((weak_import));
extern volatile u32 native_adreno_fence_wait[8] __attribute__((weak_import));
extern volatile u32 native_legacy_gpu_fallbacks __attribute__((weak_import)), native_legacy_gpu_last_status __attribute__((weak_import));
extern volatile u64 native_legacy_early_retires __attribute__((weak_import)), native_legacy_early_failures __attribute__((weak_import));
extern char native_adreno_metal_rows[48][96] __attribute__((weak_import));
// The process pass and the UI processes' stacks (BootProcessProbe.c and BootUiTraceProbe.c, UniXNU-tb128fu's).
extern void boot_process_capture(void) __attribute__((weak_import));
extern void boot_process_snapshot(void) __attribute__((weak_import));
extern char boot_process_rows[10][120] __attribute__((weak_import));
extern unsigned boot_ui_trace_role_rows(unsigned role, char rows[][120], unsigned max, unsigned detail) __attribute__((weak_import));
extern unsigned boot_ui_trace_hot_rows(char rows[][120], unsigned max) __attribute__((weak_import));
extern unsigned long vm_kernel_slide;
static void stack_row(unsigned index, const char *text) {
  static char last[8][300];
  if (!strncmp(last[index], text, sizeof last[index])) return;
  strlcpy(last[index], text, sizeof last[index]);
  line("gpu stack: ", last[index]);
}
static void gpu_stack_status(void) {
  char row[300];
  if (&native_gpu_addon_files) {
    snprintf(row, sizeof row, "addon files=%d written=%d same=%d failed=%d error=%d | mounts %d.%x %d.%x %d.%x",
             native_gpu_addon_files, native_gpu_addon_written, native_gpu_addon_same, native_gpu_addon_failed, native_gpu_addon_error,
             native_gpu_addon_stage, (unsigned)native_gpu_addon_status, native_gpu_addon2_stage, (unsigned)native_gpu_addon2_status,
             native_gpu_addon3_stage, (unsigned)native_gpu_addon3_status);
    stack_row(0, row);
  }
  if (&native_adreno_metal_diag) {
    const struct native_adreno_metal_diag_view *d = &native_adreno_metal_diag;
    const unsigned total = d->marks_total, shown = total < 8 ? total : 8;
    unsigned used = (unsigned)snprintf(row, sizeof row, "plugin opens=%u busy=%u bad=%u closes=%u namereads=%u foreign=%u/%x starter=%.16s marks=%u:",
                                       d->opens, d->busy, d->bad, d->closes, d->name_reads, d->foreign_opens, d->foreign_type, d->starter[0], total);
    for (unsigned i = 0; i < shown && used < sizeof row; ++i)
      used += (unsigned)snprintf(row + used, sizeof row - used, " %x", d->marks[(total - shown + i) & 15]);
    stack_row(1, row);
    snprintf(row, sizeof row, "plugin report %.63s | %.63s", d->report, d->report2);
    stack_row(2, row);
  }
  // Who reads MetalPluginName (NativeAdrenoPublish.cpp native_adreno_metal_askers): the plugin row's namereads= counts the reads, this
  // names the processes behind them and each one's count, so "3 reads" cannot be read as "3 processes".
  if (&native_adreno_metal_askers) {
    unsigned count = 0;
    for (unsigned i = 0; i < 12 && native_adreno_metal_askers[i].name[0]; ++i) ++count;
    if (count) {
      unsigned used = (unsigned)snprintf(row, sizeof row, "MetalPluginName askers procs=%u:", count);
      for (unsigned i = 0; i < count && i < 6 && used + 26 < sizeof row; ++i)
        used += (unsigned)snprintf(row + used, sizeof row - used, " %.19s %u", native_adreno_metal_askers[i].name,
                                   native_adreno_metal_askers[i].reads);
      if (count > 6 && used + 8 < sizeof row) snprintf(row + used, sizeof row - used, " +%u", count - 6);
      stack_row(7, row);
    }
  }
  if (&native_adreno_client_log) {
    unsigned used = (unsigned)snprintf(row, sizeof row, "clients");
    for (unsigned i = 0; i < 8 && native_adreno_client_log[i].name[0] && used < sizeof row; ++i) {
      const struct native_adreno_client_record *r = &native_adreno_client_log[i];
      used += (unsigned)snprintf(row + used, sizeof row - used, " %.19s %u/%u/%u s%d %uM r%x", r->name, r->opens, r->refused, r->closes,
                                 r->slot < 8 ? (int)r->slot : -1, r->peak_mib, r->last_refusal);
    }
    stack_row(3, row);
  }
  if (&native_adreno_open_log_count && native_adreno_open_log_count) {
    const unsigned count = native_adreno_open_log_count, shown = count < 4 ? count : 4;
    unsigned used = (unsigned)snprintf(row, sizeof row, "opens %u:", count);
    for (unsigned i = 0; i < shown && used < sizeof row; ++i) {
      const struct native_adreno_open_record *r = &native_adreno_open_log[(count - 1 - i) & 7];
      used += (unsigned)snprintf(row + used, sizeof row - used, " %.16s t%x m%x r%x a%x%s", r->name, r->type, r->macf, r->res, r->attach,
                                 r->foreign ? " F" : "");
    }
    stack_row(4, row);
  }
  if (&native_gop_wait_stats && native_gop_wait_stats[0]) {
    snprintf(row, sizeof row, "scanout wait calls=%lu timeouts=%lu asked=%lums | hand-off fallbacks %u last %x early retires %lu failures %lu",
             native_gop_wait_stats[0], native_gop_wait_stats[1], native_gop_wait_stats[2],
             &native_legacy_gpu_fallbacks ? native_legacy_gpu_fallbacks : 0, &native_legacy_gpu_last_status ? native_legacy_gpu_last_status : 0,
             &native_legacy_early_retires ? native_legacy_early_retires : 0, &native_legacy_early_failures ? native_legacy_early_failures : 0);
    stack_row(5, row);
  }
  if (&native_adreno_fence_wait && native_adreno_fence_wait[0]) {
    snprintf(row, sizeof row, "fence waits calls=%u timeouts=%u last timed-out fence=%u gpu wrote=%u submitted=%u completed=%u no-job=%u refused=%u",
             native_adreno_fence_wait[0], native_adreno_fence_wait[1], native_adreno_fence_wait[2], native_adreno_fence_wait[3],
             native_adreno_fence_wait[4], native_adreno_fence_wait[5], native_adreno_fence_wait[6], native_adreno_fence_wait[7]);
    stack_row(6, row);
  }
  // The Metal bridge's telemetry table (NativeAdrenoPublish.cpp native_adreno_metal_rows, resent by the plugin every 4 s: commit and
  // frame timings, pipeline creation, who waits for completion), as UniXNU-tb128fu's BootLog.c writes it: each row whose text changed.
  if (&native_adreno_metal_rows) {
    static char last_rows[48][96];
    for (unsigned r = 0; r < 48; ++r) {
      if (!native_adreno_metal_rows[r][0] || !strncmp(last_rows[r], native_adreno_metal_rows[r], sizeof last_rows[r])) continue;
      strlcpy(last_rows[r], native_adreno_metal_rows[r], sizeof last_rows[r]);
      line("metal table: ", last_rows[r]);
    }
  }
}

// The last complete process pass (BootProcessProbe.c: per process its threads, running, waiting, the first thread's state, priority,
// wait event, continuation and user PC), each row whose text changed; with stacks, also backboardd's and SpringBoard's threads
// (BootUiTraceProbe.c: the main thread's registers and frames, every thread group's Mach peer, PC, LR and frames, the busiest
// threads with their kernel frames), as UniXNU-tb128fu's BootLog.c writes them. User addresses are raw (cache+ is the shared cache
// slide), kernel ones slid by the kernel slide. Two rounds of 2026-10-24 stopped rendering after the first frames without a crash.
static void process_status(int stacks) {
  if (!&boot_process_snapshot || !&boot_process_rows) return;
  boot_process_snapshot();
  static char last[10][120];
  for (unsigned i = 0; i < 10; ++i) {
    if (!boot_process_rows[i][0] || !strncmp(last[i], boot_process_rows[i], sizeof last[i])) continue;
    strlcpy(last[i], boot_process_rows[i], sizeof last[i]);
    line("process: ", last[i]);
  }
  if (!stacks) return;
  static char rows[32][120];
  char head[64];
  snprintf(head, sizeof head, "kernel slide %lx", vm_kernel_slide);
  line("stacks: ", head);
  if (&boot_ui_trace_role_rows)
    for (unsigned role = 0; role < 4; ++role) {
      const unsigned n = boot_ui_trace_role_rows(role, rows, 32, 1);
      for (unsigned i = 0; i < n; ++i) line("stacks: ", rows[i]);
    }
  if (&boot_ui_trace_hot_rows) {
    const unsigned n = boot_ui_trace_hot_rows(rows, 16);
    for (unsigned i = 0; i < n; ++i) line("stacks: ", rows[i]);
  }
}
// The process pass can wait in proc_iterate and its stack reads can fault in user pages: it runs on a worker of its own every 2 s,
// from 60 s after the log zone is mapped (the UI processes exist by then); the writer only copies the last complete pass.
static void process_worker(void *argument, int result) {
  (void)argument; (void)result;
  IOSleep(60000);
  for (;;) {
    boot_process_capture();
    IOSleep(2000);
  }
}

// What the frame buffer holds, through the display driver's own kernel mapping of it (the one its CPU copies write; a second
// mapping of the 20 MiB panic'd: kmem_alloc failed with 3): a 32 x 48 grid of pixels spread over the panel, how many are not
// black and a sum of them, written when it changes (every 10 s and before a restart). With the GPU hand-off the plugin's copies
// verified against the surfaces while the panel stayed black (2026-10-24): this tells black frames from frames the panel does not show.
extern volatile u64 native_legacy_gop_physical __attribute__((weak_import)), native_legacy_gop_bytes __attribute__((weak_import)),
    native_legacy_gop_virtual __attribute__((weak_import));
static void framebuffer_status(void) {
  if (!&native_legacy_gop_physical || !&native_legacy_gop_bytes || !&native_legacy_gop_virtual || !&native_legacy_gop_width ||
      !&native_legacy_gop_height) return;
  const u64 phys = native_legacy_gop_physical, bytes = native_legacy_gop_bytes;
  const u32 width = native_legacy_gop_width, height = native_legacy_gop_height;
  volatile const u32 *fb = (volatile const u32 *)native_legacy_gop_virtual;
  static char last[120];
  char text[120];
  if (!fb || !width || !height || bytes < (u64)width * 4 * height) {
    snprintf(text, sizeof text, "not sampled: mapping %lx bytes %lx at %lx, %ux%u", phys, bytes, (u64)fb, width, height);
    if (strncmp(last, text, sizeof last)) {strlcpy(last, text, sizeof last); line("framebuffer: ", text);}
    return;
  }
  u32 lit = 0, sum = 0, first = 0;
  for (u32 gy = 0; gy < 48; ++gy) {
    for (u32 gx = 0; gx < 32; ++gx) {
      const u32 x = (gx * 2 + 1) * width / 64, y = (gy * 2 + 1) * height / 96;
      const u32 pixel = fb[(u64)y * width + x];
      if (pixel & 0x00ffffffu) {if (!lit) first = pixel; ++lit;}
      sum = sum * 31 + pixel;
    }
  }
  snprintf(text, sizeof text, "%u of 1536 samples not black, sum %08x, first lit %08x (fb %lx, %ux%u)", lit, sum, first, phys,
           width, height);
  if (!strncmp(last, text, sizeof last)) return;
  strlcpy(last, text, sizeof last);
  line("framebuffer: ", text);
  // The same frame as a 45 x 36 picture of its luma (one character per 40 x 80 pixels, " .:-=+*#%@" from dark to light), to tell
  // the screens apart in a round nobody watches: the scripted strokes' rounds differed by the screens they turned to (2026-10-24).
  static const char ramp[] = " .:-=+*#%@";
  for (u32 cy = 0; cy < 36; ++cy) {
    char row[48];
    for (u32 cx = 0; cx < 45; ++cx) {
      const u32 x = (cx * 2 + 1) * width / 90, y = (cy * 2 + 1) * height / 72;
      const u32 pixel = fb[(u64)y * width + x];
      const u32 luma = (((pixel >> 16) & 0xff) * 77 + ((pixel >> 8) & 0xff) * 150 + (pixel & 0xff) * 29) >> 8;
      row[cx] = ramp[luma * 10 / 256];
    }
    row[45] = 0;
    line("fbpic: ", row);
  }
}

// Two 64 x 64 pixel crops of the frame buffer at full resolution, one character per pixel (the same luma ramp): the home screen's first
// widget's top-left corner, and a run of the dock's label text. The user saw every rounded corner of the Metal-rendered UI drawn wrong
// (2026-10-24); the text says whether the whole frame shifts in 4-row slices or only the corner's shape does.
static const char kLumaRamp[] = " .:-=+*#%@";

static char lumaChar(volatile const u32 *fb, u32 width, u32 x, u32 y) {
  const u32 pixel = fb[(u64)y * width + x];
  const u32 luma = (((pixel >> 16) & 0xff) * 77 + ((pixel >> 8) & 0xff) * 150 + (pixel & 0xff) * 29) >> 8;
  return kLumaRamp[luma * 10 / 256];
}

static void framebuffer_picture(volatile const u32 *fb, u32 width, u32 height, u32 x0, u32 y0, u32 step) {
  char head[64];
  snprintf(head, sizeof head, "%ux%u of %u,%u every %u", 64 * step, 64 * step, x0, y0, step);
  line("fbcrop: ", head);
  for (u32 y = 0; y < 64; ++y) {
    char row[72];
    for (u32 x = 0; x < 64; ++x) {
      const u32 px = x0 + x * step < width ? x0 + x * step : width - 1;
      const u32 py = y0 + y * step < height ? y0 + y * step : height - 1;
      row[x] = lumaChar(fb, width, px, py);
    }
    row[64] = 0;
    line("fbcrop: ", row);
  }
}

static void framebuffer_crops(void) {
  if (!&native_legacy_gop_virtual || !&native_legacy_gop_width || !&native_legacy_gop_height || !&native_legacy_gop_bytes) return;
  const u32 width = native_legacy_gop_width, height = native_legacy_gop_height;
  volatile const u32 *fb = (volatile const u32 *)native_legacy_gop_virtual;
  if (!fb || width < 256 || height < 1024 || native_legacy_gop_bytes < (u64)width * 4 * height) return;
  static const char ramp[] = " .:-=+*#%@";
  // The first widget's top-left corner (first page) and a run of the dock's label text (a glyph's edge tells a whole-frame 4-row
  // shift from one that only the corner's shape suffers).
  // The first widget's corner, found rather than fixed: the first row below the status bar with a long bright run is the platter's top
  // edge, so the crop follows the layout (2026-10-25: a fixed origin missed it after the UI moved by a few dozen pixels).
  u32 located[2] = {176, 136};
  for (u32 y = 120; y + 8 < height && located[1] == 136; ++y) {
    for (u32 x = 96; x + 40 < width; ++x) {
      u32 run = 0;
      while (run < 40 && lumaChar(fb, width, x + run, y) == '@') ++run;
      if (run == 40) {located[0] = x > 40 ? x - 40 : 0; located[1] = y > 40 ? y - 40 : 0; break;}
    }
    if (y > 900) break;
  }
  const u32 origin[2][2] = {{located[0], located[1]}, {560, 2740}};
  for (u32 c = 0; c < 2; ++c) {
    // The same corner with 256 x 256 pixels of its surroundings, four pixels per character, to see what the detail crop is a part of.
    if (c == 0) framebuffer_picture(fb, width, height, origin[c][0] - 80, origin[c][1] - 56, 4);
    char head[48];
    snprintf(head, sizeof head, "%ux%u at %u,%u", 64, 64, origin[c][0], origin[c][1]);
    line("fbcrop: ", head);
    for (u32 y = 0; y < 64; ++y) {
      char row[72];
      for (u32 x = 0; x < 64; ++x) {
        const u32 pixel = fb[(u64)(origin[c][1] + y) * width + origin[c][0] + x];
        const u32 luma = (((pixel >> 16) & 0xff) * 77 + ((pixel >> 8) & 0xff) * 150 + (pixel & 0xff) * 29) >> 8;
        row[x] = ramp[luma * 10 / 256];
      }
      row[64] = 0;
      line("fbcrop: ", row);
    }
    // The crop's first lit column per row (two hex digits, 255 = none) and how many pixels differ from the row above (one hex digit, '.' for
    // none): a shape whose columns and rows repeat in groups of four is the 4 x 4 granularity of the GMEM path showing through, where the
    // plain picture alone cannot tell a coarse edge from a scrambled one.
    char edges[144], deltas[80];
    for (u32 y = 0; y < 64; ++y) {
      u32 edge = 255, diff = 0;
      for (u32 x = 0; x < 64; ++x) {
        const u32 pixel = fb[(u64)(origin[c][1] + y) * width + origin[c][0] + x];
        const u32 luma = (((pixel >> 16) & 0xff) * 77 + ((pixel >> 8) & 0xff) * 150 + (pixel & 0xff) * 29) >> 8;
        if (edge == 255 && luma >= 160) edge = x;
        if (y) {
          const u32 above = fb[(u64)(origin[c][1] + y - 1) * width + origin[c][0] + x];
          if (above != pixel) ++diff;
        }
      }
      snprintf(edges + y * 2, 4, "%02x", edge);
      deltas[y] = y == 0 ? '.' : (diff == 0 ? '.' : "123456789abcdef"[(diff > 15 ? 15 : diff) - 1]);
    }
    edges[128] = 0;
    deltas[64] = 0;
    line("fbcrop: edge ", edges);
    line("fbcrop: diff ", deltas);
    // The same pixels read again a moment later: a count that is not zero means the frame buffer was being written while it was read (the
    // GPU's copy of the next frame), so the crop is a tear and not what the panel showed.
    static u32 snapshot[64 * 64];
    for (u32 y = 0; y < 64; ++y)
      for (u32 x = 0; x < 64; ++x) snapshot[y * 64 + x] = fb[(u64)(origin[c][1] + y) * width + origin[c][0] + x];
    u32 moved = 0;
    for (u32 y = 0; y < 64; ++y)
      for (u32 x = 0; x < 64; ++x)
        if (snapshot[y * 64 + x] != fb[(u64)(origin[c][1] + y) * width + origin[c][0] + x]) ++moved;
    char again[64];
    snprintf(again, sizeof again, "%u pixels differ from a second read", moved);
    line("fbcrop: reread ", again);
    // The first crop's luma as two hex digits per pixel (64 rows of 128 characters): a round with GMEM and one without hold the same
    // screen, so the two dumps tell a permutation of rows from content that was never drawn finer than a block.
    if (c == 0) {
      for (u32 y = 0; y < 64; ++y) {
        char hex[136];
        for (u32 x = 0; x < 64; ++x) {
          const u32 pixel = fb[(u64)(origin[c][1] + y) * width + origin[c][0] + x];
          const u32 luma = ((pixel >> 16) & 0xff) * 77 + ((pixel >> 8) & 0xff) * 150 + (pixel & 0xff) * 29;
          const char *digits = "0123456789abcdef";
          hex[x * 2] = digits[(luma >> 12) & 0xf];
          hex[x * 2 + 1] = digits[(luma >> 8) & 0xf];
        }
        hex[128] = 0;
        line("fbcrop: h ", hex);
      }
    }
  }
}

// The lines sheng's progress page shows for other components (KeyStoreProbe.cpp writes line 0); logged with the display line.
char boot_progress_ext[8][48];

// One line of the display's own state, written when its text changes (the counters in it move every sample while the UI runs).
static void display_row(unsigned slot, const char *text) {
  static char last[2][224];
  if (slot > 1 || !text[0] || !strncmp(last[slot], text, sizeof last[slot])) return;
  strlcpy(last[slot], text, sizeof last[slot]);
  line("display: ", last[slot]);
}

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
  // The frame rate while the UI moves (LegacyDisplayProbe.cpp): swaps less than 200 ms apart, their gaps, submit-to-retire.
  if (&native_legacy_swap_pacing && native_legacy_swap_pacing(text, sizeof text)) line("display: ", text);
  // The scanout hand-off's retries and whether the refresh tick still runs (LegacyDisplayProbe.cpp): the two facts a round that draws
  // one or two frames and then nothing is decided by. A tick that stopped shows sent frozen and the age of its last fire growing,
  // while wants stays 1; an idle stop shows idle counting up instead.
  if (&native_legacy_prepare_brief && native_legacy_prepare_brief(text, sizeof text)) display_row(0, text);
  if (&native_legacy_vsync_brief && native_legacy_vsync_brief(text, sizeof text)) display_row(1, text);
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
    if (pass % 100 == 0) {display_status(); cpu_status(); touch_status(); gpu_stack_status(); framebuffer_status(); process_status(0); ddr_status();}
    if (pass == 2000) process_status(1);   // the UI's stacks 200 s after the zone was mapped
    // Two corner crops every 50 s: a round can end on a panic before the 200 s mark (2026-10-24's unaligned_store round did), and
    // the crops of a round that turned the pages are the ones worth reading.
    if (pass % 500 == 0) framebuffer_crops();
    if (pass % 5 == 0) touch_strokes();
    buttons_status();
    battery_status();
    gesture_status();
    gpu_status(pass % 100 == 0);
    mdss_status(pass % 100 == 0);
    flip_status();
    // The refresh tick's watchdog (NativeLegacyDisplay.cpp): a tick that stopped while the render server waits for it never comes back
    // by itself (the family arms it from kickSwapQueue, and a client waiting for a tick submits no swap), so ask every pass. While
    // ticks arrive this reads two words; when it arms one again the display rows' re= count shows it.
    if (&native_legacy_vsync_kick) native_legacy_vsync_kick();
    if (restart_after && ticks() - start >= limit) {
      display_status();
      gpu_stack_status();
      framebuffer_status();
      process_status(1);
      framebuffer_crops();
      char text[64];
      snprintf(text, sizeof text, "%u s after the zone was mapped (pipa-restart-after)", restart_after);
      line("restart into fastboot: ", text);
      restart_into_fastboot();
    }
    if (restart_crashes && ui_crashes >= restart_crashes) {
      messages();
      framebuffer_crops();
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
  watchdog_start();
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
  if (&boot_process_capture && kernel_thread_start(process_worker, 0, &thread) == 0) thread_deallocate(thread);
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
