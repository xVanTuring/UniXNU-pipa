// SPDX-License-Identifier: BSD-2-Clause-Patent
// The kernel's log in the stock ramoops console zone, after the firmware's (uefi-pipa's RamoopsLogLib writes the same zone): the
// boot_progress_* hooks of zzz-boot-progress.patch (XNU_BOOT_PROGRESS) without a progress page. Stages and process events are
// written as they come; from boot_progress_start on, a thread appends the message buffer's new text every 100 ms. The zone
// survives the warm reset the firmware makes the power key's long press (and power with volume down) do, and the log reader
// (package/linux-log, scripts/read-pipa-log.py) reads it back as console-ramoops-0. The console stays XNU's (no takeover), so the
// panic screen is unchanged; what the panic itself prints after the last pass is not in the zone.
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

extern u64 ml_io_map(u64 phys_addr, u64 size);
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

static volatile u8 *zone;                       // the zone's mapping, once boot_progress_start made it
static char early[EARLY_SIZE];                  // what came before it
static u32 early_len;
static volatile int lock;
static u32 user_faults;

static void take(void) {while (__atomic_exchange_n(&lock, 1, __ATOMIC_ACQUIRE)) {}}
static void give(void) {__atomic_store_n(&lock, 0, __ATOMIC_RELEASE);}

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
  take();
  if (zone) zone_put(text, n);
  else for (unsigned i = 0; i < n && early_len < EARLY_SIZE; ++i) early[early_len++] = text[i];
  give();
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

static void writer(void *argument, int result) {
  (void)argument; (void)result;
  for (;;) {
    messages();
    IOSleep(PASS_MS);
  }
}

void boot_progress_init(void) {line("", "boot_progress_init (stages kept until the ramoops zone is mapped)");}

void boot_progress_start(void) {
  volatile u8 *mapped = (volatile u8 *)ml_io_map(ZONE_BASE, ZONE_SIZE);
  if (!mapped) return;
  take();
  zone = mapped;
  static const char banner[] = "\n[xnu] ---- kernel: ramoops console zone mapped; the stages before it follow ----\n";
  zone_put(banner, sizeof banner - 1);
  zone_put(early, early_len);
  give();
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
