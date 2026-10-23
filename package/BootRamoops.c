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
// boot_progress_start (0 = never), and pipa-panic-restart-after=<s> (default 10) after a panic. The restart is uefi-pipa's
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
void boot_progress_user_breakpoint(void *, unsigned);
void boot_progress_user_abort(void *, unsigned, u64);
void boot_progress_critical_exit(int, char *, unsigned long);
void boot_progress_process_stage(int, const char *, const char *);
void boot_progress_exit(int, const char *, int, unsigned, u64);
void boot_progress_panic_hook(void);

static volatile u8 *zone;                       // the zone's mapping, once boot_progress_start made it
static char early[EARLY_SIZE];                  // what came before it
static u32 early_len;
static volatile int lock;
static u32 user_faults;
static volatile u32 *ps_hold, *imem;            // the restart's registers, mapped by boot_progress_start
static u32 restart_after, panic_restart_after = 10;
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
  __asm__ volatile("dsb sy" ::: "memory");
}

static void put(const char *text, unsigned n) {
  u64 daif = take();
  if (zone) zone_put(text, n);
  else for (unsigned i = 0; i < n && early_len < EARLY_SIZE; ++i) early[early_len++] = text[i];
  give(daif);
}

static void line(const char *prefix, const char *text) {
  char buffer[320];
  int n = snprintf(buffer, sizeof buffer, "[xnu] %s%s\n", prefix, text ? text : "");
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

static void writer(void *argument, int result) {
  (void)argument; (void)result;
  const u64 start = ticks(), limit = ticks_per_second() * restart_after;
  for (;;) {
    messages();
    panic_text(0);
    if (restart_after && ticks() - start >= limit) {
      char text[64];
      snprintf(text, sizeof text, "%u s after the zone was mapped (pipa-restart-after)", restart_after);
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
  u32 pon = prepare_warm_fastboot();
  ps_hold = (volatile u32 *)ml_io_map(PS_HOLD, 0x1000);
  volatile u8 *imem_page = (volatile u8 *)ml_io_map(IMEM_RESTART_REASON & ~0xFFFUL, 0x1000);
  imem = imem_page ? (volatile u32 *)(imem_page + (IMEM_RESTART_REASON & 0xFFF)) : 0;
  char text[128];
  snprintf(text, sizeof text, "PON 0x%x (%s), PS_HOLD %s, after %u s, after a panic %u s", pon, pon ? "not prepared" : "warm, bootloader",
           ps_hold && imem ? "mapped" : "NOT mapped", restart_after, panic_restart_after);
  line("restart into fastboot: ", text);
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

void boot_progress_user_breakpoint(void *state, unsigned esr) {
  (void)state;
  if (user_faults >= 64) return;
  ++user_faults;
  char text[64];
  snprintf(text, sizeof text, "esr 0x%x", esr);
  line("user breakpoint: ", text);
}

void boot_progress_user_abort(void *state, unsigned esr, u64 address) {
  (void)state;
  if (user_faults >= 64) return;
  ++user_faults;
  char text[80];
  snprintf(text, sizeof text, "esr 0x%x address 0x%llx", esr, (unsigned long long)address);
  line("user abort: ", text);
}

void boot_progress_critical_exit(int pid, char *text, unsigned long capacity) {
  (void)capacity;
  char prefix[48];
  snprintf(prefix, sizeof prefix, "critical exit %d: ", pid);
  line(prefix, text);
}

void boot_progress_process_stage(int pid, const char *name, const char *stage) {
  char prefix[96];
  snprintf(prefix, sizeof prefix, "%d %s: ", pid, name ? name : "?");
  line(prefix, stage);
}

void boot_progress_exit(int pid, const char *name, int status, unsigned reason_namespace, u64 reason_code) {
  char text[160];
  snprintf(text, sizeof text, "%d %s status 0x%x reason %u/0x%llx", pid, name ? name : "?", status, reason_namespace,
           (unsigned long long)reason_code);
  line("exit ", text);
}
