// SPDX-License-Identifier: MIT
// pipa log reader init (see build-log-reader.py). The stock kernel reserves a 4 MiB ramoops area at 0xb0000000 (its built-in
// command line ramoops_memreserve=4M: 2 MiB console, 2 MiB pmsg, no ECC) and, when the memory survived the restart, shows what
// the previous system left there under /sys/fs/pstore. This program copies those files, this kernel's own log and its command
// line into LOG.BIN of the FAT image /log.img, writes a marker with this boot's id into pmsg and the kernel log, and offers the
// image read only as a USB mass storage disk; STATUS.TXT in the same image says what it did. When the host ejects the disk, or
// after TIMEOUT_S, it restarts into fastboot: a warm reset when the reader's device tree has qcom,force-warm-reboot, so the next
// reader finds this one's marker if the memory survives the way back. Static and libc-free; nothing on UFS is read or written.
// Adapted from the TB128FU reader (UniXNU-tb128fu package/linux-log/init.c).

// The host mounts and copies within about 15 s of boot. Some ejects were not seen (2026-10-24, twice: the LUN kept its file), and
// the tablet came back to fastboot 8.5 to 9 minutes after the reader started, so 300 rounds of this wait take far longer than
// 300 s; 60 rounds are enough.
#define TIMEOUT_S  60
#define BIG        (4 * 1024 * 1024)
#define GADGET     "/sys/kernel/config/usb_gadget/log"
#define LUN        GADGET "/functions/mass_storage.0/lun.0"
#define PSTORE     "/sys/fs/pstore"

enum {
  SysMknodat = 33, SysMkdirat = 34, SysSymlinkat = 36, SysMount = 40, SysOpenat = 56, SysClose = 57, SysGetdents64 = 61,
  SysRead = 63, SysWrite = 64, SysPwrite64 = 68, SysSync = 81, SysExit = 93, SysNanosleep = 101, SysSyslog = 116, SysReboot = 142,
};
enum { AtFdcwd = -100, ORdonly = 0, OWronly = 1, ORdwr = 2, SyslogReadAll = 3 };

static long sys(long n, long a, long b, long c, long d, long e)
{
  register long x8 __asm__("x8") = n, x0 __asm__("x0") = a, x1 __asm__("x1") = b, x2 __asm__("x2") = c,
                x3 __asm__("x3") = d, x4 __asm__("x4") = e;
  __asm__ volatile("svc #0" : "+r"(x0) : "r"(x8), "r"(x1), "r"(x2), "r"(x3), "r"(x4) : "memory");
  return x0;
}

// The compiler may emit calls to these; the volatile stores keep it from turning the loops back into calls.
void *memset(void *d, int c, unsigned long n)
{
  volatile unsigned char *p = d;
  while (n--) *p++ = (unsigned char)c;
  return d;
}
void *memcpy(void *d, const void *s, unsigned long n)
{
  volatile unsigned char *p = d;
  const unsigned char *q = s;
  while (n--) *p++ = *q++;
  return d;
}

static unsigned long slen(const char *s) { unsigned long n = 0; while (s[n]) ++n; return n; }

// Offset of NEEDLE in HAY, or -1.
static long find(const char *hay, unsigned long len, const char *needle)
{
  unsigned long n = slen(needle);
  for (unsigned long i = 0; i + n <= len; ++i) {
    unsigned long j = 0;
    while (j < n && hay[i + j] == needle[j]) ++j;
    if (j == n) return (long)i;
  }
  return -1;
}

static void sleep_s(long seconds) { long ts[2] = {seconds, 0}; sys(SysNanosleep, (long)ts, 0, 0, 0, 0); }

// STATUS.TXT: one line per step, rewritten into the image after every step.
static char status[4096];
static unsigned long status_len, status_off, status_cap;
static int image = -1;

static void put_n(const char *s, unsigned long n)
{
  if (status_len + n < sizeof(status)) { memcpy(status + status_len, s, n); status_len += n; }
}
static void put(const char *s) { put_n(s, slen(s)); }
static void put_num(unsigned long v)
{
  char b[24]; int i = 23; b[i] = 0;
  do { b[--i] = (char)('0' + v % 10); v /= 10; } while (v);
  put(b + i);
}
static void flush_status(void)
{
  if (image < 0 || !status_cap) return;
  static char page[4096];
  unsigned long n = status_cap < sizeof(page) ? status_cap : sizeof(page);
  memset(page, ' ', n);
  memcpy(page, status, status_len < n ? status_len : n);
  page[n - 1] = '\n';
  sys(SysPwrite64, image, (long)page, (long)n, (long)status_off, 0);
}
static void step(const char *what, long result)
{
  put(what); put(" = "); if (result < 0) { put("-"); put_num((unsigned long)-result); } else put_num((unsigned long)result);
  put("\n"); flush_status();
}

static long write_file(const char *path, const char *text)
{
  long fd = sys(SysOpenat, AtFdcwd, (long)path, OWronly, 0, 0);
  if (fd < 0) return fd;
  long r = sys(SysWrite, fd, (long)text, (long)slen(text), 0, 0);
  sys(SysClose, fd, 0, 0, 0, 0);
  return r;
}
static long read_file(const char *path, char *buf, unsigned long cap)
{
  long fd = sys(SysOpenat, AtFdcwd, (long)path, ORdonly, 0, 0);
  if (fd < 0) return fd;
  long r = sys(SysRead, fd, (long)buf, (long)cap - 1, 0, 0);
  sys(SysClose, fd, 0, 0, 0, 0);
  buf[r > 0 ? r : 0] = 0;
  return r;
}
// The whole file, up to CAP bytes (pstore files are read in pieces).
static long read_all(const char *path, char *buf, unsigned long cap)
{
  long fd = sys(SysOpenat, AtFdcwd, (long)path, ORdonly, 0, 0);
  if (fd < 0) return fd;
  unsigned long got = 0;
  while (got < cap) {
    long r = sys(SysRead, fd, (long)(buf + got), (long)(cap - got), 0, 0);
    if (r < 0) { sys(SysClose, fd, 0, 0, 0, 0); return r; }
    if (r == 0) break;
    got += (unsigned long)r;
  }
  sys(SysClose, fd, 0, 0, 0, 0);
  return (long)got;
}
static unsigned long parse_num(const char **p)
{
  unsigned long v = 0;
  while (**p == ' ' || **p == '\n') ++*p;
  while (**p >= '0' && **p <= '9') v = v * 10 + (unsigned long)(*(*p)++ - '0');
  return v;
}

static void join(char *out, const char *a, const char *b)
{
  unsigned long n = slen(a), m = slen(b);
  memcpy(out, a, n); memcpy(out + n, b, m + 1);
}

// The first entry of DIR (other than . and ..) copied into NAME; 1 if found.
static int first_entry(const char *dir, char *name, unsigned long cap)
{
  static char ents[4096];
  long fd = sys(SysOpenat, AtFdcwd, (long)dir, ORdonly, 0, 0);
  if (fd < 0) return 0;
  int found = 0;
  long n = sys(SysGetdents64, fd, (long)ents, sizeof(ents), 0, 0);
  for (long at = 0; at < n && !found;) {
    const char *entry = ents + at + 19;
    if (entry[0] != '.' && slen(entry) < cap) { memcpy(name, entry, slen(entry) + 1); found = 1; }
    at += *(unsigned short *)(ents + at + 16);
  }
  sys(SysClose, fd, 0, 0, 0, 0);
  return found;
}

// LOG.BIN: per item a 64-byte record header ("UXFILE01", the size or a negative errno as a signed 64-bit number, the name, zero
// padded), then the data padded to 8 bytes; "UXEND000" after the last (the FIRMWARE.BIN format of the TB128FU reader).
static unsigned long log_off, log_cap, log_at;
static long record(const char *name, const char *data, long size)
{
  char head[64];
  unsigned long n = slen(name);
  if (log_at + 64 + 8 > log_cap) return -27;
  if (size > 0 && log_at + 64 + (unsigned long)size + 8 > log_cap) size = -27;
  memset(head, 0, sizeof(head));
  memcpy(head, "UXFILE01", 8);
  for (int i = 0; i < 8; ++i) head[8 + i] = (char)((unsigned long)size >> (8 * i));
  memcpy(head + 16, name, n < 47 ? n : 47);
  sys(SysPwrite64, image, (long)head, 64, (long)(log_off + log_at), 0);
  if (size > 0) sys(SysPwrite64, image, (long)data, size, (long)(log_off + log_at + 64), 0);
  log_at += 64 + (size > 0 ? ((unsigned long)size + 7) / 8 * 8 : 0);
  sys(SysPwrite64, image, (long)"UXEND000", 8, (long)(log_off + log_at), 0);
  return size;
}

static char big[BIG];

// Every file pstore shows (console-ramoops-0, pmsg-ramoops-0, dmesg-ramoops-N: what the previous system left), one record each.
static long copy_pstore(void)
{
  static char ents[4096];
  long fd = sys(SysOpenat, AtFdcwd, (long)PSTORE, ORdonly, 0, 0);
  if (fd < 0) return fd;
  long n = sys(SysGetdents64, fd, (long)ents, sizeof(ents), 0, 0);
  sys(SysClose, fd, 0, 0, 0, 0);
  if (n < 0) return n;
  long count = 0;
  for (long at = 0; at < n;) {
    const char *entry = ents + at + 19;
    at += *(unsigned short *)(ents + at + 16);
    if (entry[0] == '.' || slen(entry) > 100) continue;
    char path[128];
    join(path, PSTORE "/", entry);
    long size = read_all(path, big, BIG);
    char name[64];
    join(name, "pstore/", entry);
    step(name, record(name, big, size));
    if (size > 0 && find(big, (unsigned long)size, "UNIXNU-PIPA-READER") >= 0) put("  holds a reader marker\n");
    ++count;
  }
  return count;
}

// /dev/pmsg0: the stock kernel has no devtmpfs, and pmsg's major is dynamic (252 under Android).
static long open_pmsg(void)
{
  static char devices[8192];
  long fd = sys(SysOpenat, AtFdcwd, (long)"/dev/pmsg0", OWronly, 0, 0);
  if (fd >= 0) return fd;
  long r = read_file("/proc/devices", devices, sizeof(devices));
  if (r <= 0) return r < 0 ? r : -2;
  long at = find(devices, (unsigned long)r, " pmsg\n");
  if (at < 0) return -2;
  long line = at;
  while (line > 0 && devices[line - 1] != '\n') --line;
  const char *p = devices + line;
  unsigned long major = parse_num(&p);
  put("pmsg_major="); put_num(major); put("\n");
  step("mknod.pmsg0", sys(SysMknodat, AtFdcwd, (long)"/dev/pmsg0", 020600, (long)(major << 8), 0));
  return sys(SysOpenat, AtFdcwd, (long)"/dev/pmsg0", OWronly, 0, 0);
}

// "UNIXNU-PIPA-READER boot_id=<this boot's id>" into pmsg (pmsg-ramoops-0 of the next boot) and the kernel log (its
// console-ramoops-0).
static long write_marker(void)
{
  char id[64], text[128];
  long r = read_file("/proc/sys/kernel/random/boot_id", id, sizeof(id));
  if (r <= 0) return r < 0 ? r : -5;
  if (id[r - 1] == '\n') id[r - 1] = 0;
  join(text, "UNIXNU-PIPA-READER boot_id=", id);
  join(text + slen(text), "", "\n");
  put("marker: "); put(text);
  long pmsg = open_pmsg();
  step("pmsg.open", pmsg);
  if (pmsg >= 0) { step("pmsg.write", sys(SysWrite, pmsg, (long)text, (long)slen(text), 0, 0)); sys(SysClose, pmsg, 0, 0, 0, 0); }
  long kmsg = sys(SysOpenat, AtFdcwd, (long)"/dev/kmsg", OWronly, 0, 0);
  if (kmsg < 0) {
    sys(SysMknodat, AtFdcwd, (long)"/dev/kmsg", 020600, (1 << 8) | 11, 0);
    kmsg = sys(SysOpenat, AtFdcwd, (long)"/dev/kmsg", OWronly, 0, 0);
  }
  step("kmsg.open", kmsg);
  if (kmsg >= 0) { step("kmsg.write", sys(SysWrite, kmsg, (long)text, (long)slen(text), 0, 0)); sys(SysClose, kmsg, 0, 0, 0, 0); }
  return pmsg < 0 ? pmsg : 0;
}

// This kernel's log as a record; the lines that tell how the tablet came up go to STATUS.TXT as well.
static long copy_dmesg(void)
{
  static const char *const needles[] = {"msm_reserve_ramoops_memory addr", "ramoops: attached", "pstore: ",
                                        "PMIC@SID0 Power-on reason", "PMIC@SID0: Power-off reason"};
  long n = sys(SysSyslog, SyslogReadAll, (long)big, BIG, 0, 0);
  if (n <= 0) return record("dmesg", big, n < 0 ? n : -61);
  for (unsigned i = 0; i < sizeof(needles) / sizeof(needles[0]); ++i) {
    long at = find(big, (unsigned long)n, needles[i]);
    if (at < 0) continue;
    long end = at;
    while (end < n && big[end] != '\n' && end - at < 200) ++end;
    put("  "); put_n(big + at, (unsigned long)(end - at)); put("\n");
  }
  return record("dmesg", big, n);
}

static long gadget(void)
{
  static const char *const dirs[] = {GADGET, GADGET "/strings/0x409", GADGET "/configs/c.1",
                                     GADGET "/configs/c.1/strings/0x409", GADGET "/functions/mass_storage.0"};
  for (unsigned i = 0; i < sizeof(dirs) / sizeof(dirs[0]); ++i) {
    long r = sys(SysMkdirat, AtFdcwd, (long)dirs[i], 0755, 0, 0);
    if (r < 0 && r != -17) return r;
  }
  write_file(GADGET "/idVendor", "0x1d6b");
  write_file(GADGET "/idProduct", "0x0104");
  write_file(GADGET "/strings/0x409/manufacturer", "UniXNU");
  write_file(GADGET "/strings/0x409/product", "pipa log reader");
  write_file(GADGET "/strings/0x409/serialnumber", "unixnulog");
  write_file(GADGET "/configs/c.1/strings/0x409/configuration", "log");
  // ro and removable before file: the attributes cannot change while a backing file is open.
  step("lun.removable", write_file(LUN "/removable", "1"));
  step("lun.ro", write_file(LUN "/ro", "1"));
  step("lun.file", write_file(LUN "/file", "/log.img"));
  long r = sys(SysSymlinkat, (long)(GADGET "/functions/mass_storage.0"), AtFdcwd, (long)(GADGET "/configs/c.1/mass_storage.0"), 0, 0);
  step("config.link", r);
  if (r < 0 && r != -17) return r;
  char udc[64];
  int found = 0;
  for (int i = 0; i < 20 && !found; ++i) { found = first_entry("/sys/class/udc", udc, sizeof(udc)); if (!found) sleep_s(1); }
  if (!found) return -19;
  put("udc="); put(udc); put("\n");
  r = write_file(GADGET "/UDC", udc);
  step("udc.bind", r);
  // dwc3-msm leaves peripheral mode to the charger's VBUS report; a forced mode covers a cable already present at boot.
  step("ssusb.mode", write_file("/sys/bus/platform/devices/a600000.ssusb/mode", "peripheral"));
  return r;
}

static void reboot_bootloader(void)
{
  sys(SysSync, 0, 0, 0, 0, 0);
  sys(SysReboot, (long)0xfee1dead, 672274793, (long)0xa1b2c3d4, (long)"bootloader", 0);
}

__attribute__((noreturn)) void start(void)
{
  sys(SysMkdirat, AtFdcwd, (long)"/dev", 0755, 0, 0);
  sys(SysMkdirat, AtFdcwd, (long)"/sys", 0755, 0, 0);
  sys(SysMkdirat, AtFdcwd, (long)"/proc", 0755, 0, 0);
  long devfs = sys(SysMount, (long)"devtmpfs", (long)"/dev", (long)"devtmpfs", 0, 0);
  long sysfs = sys(SysMount, (long)"sysfs", (long)"/sys", (long)"sysfs", 0, 0);
  long procfs = sys(SysMount, (long)"proc", (long)"/proc", (long)"proc", 0, 0);
  long cfs = sys(SysMount, (long)"configfs", (long)"/sys/kernel/config", (long)"configfs", 0, 0);
  long psfs = sys(SysMount, (long)"pstore", (long)PSTORE, (long)"pstore", 0, 0);

  // /log.offsets (build-log-reader.py): byte offset and size of LOG.BIN, then of STATUS.TXT inside /log.img.
  char spec[128];
  read_file("/log.offsets", spec, sizeof(spec));
  const char *p = spec;
  log_off = parse_num(&p); log_cap = parse_num(&p);
  status_off = parse_num(&p); status_cap = parse_num(&p);
  image = (int)sys(SysOpenat, AtFdcwd, (long)"/log.img", ORdwr, 0, 0);

  char release[160];
  put("pipa log reader\n");
  if (read_file("/proc/version", release, sizeof(release)) > 0) put(release);
  if (status_len && status[status_len - 1] != '\n') put("\n");
  step("mount.devtmpfs", devfs);
  step("mount.sysfs", sysfs);
  step("mount.proc", procfs);
  step("mount.configfs", cfs);
  step("mount.pstore", psfs);
  step("image.open", image);
  step("pstore.files", copy_pstore());
  step("marker", write_marker());
  long cmdline = read_all("/proc/cmdline", big, BIG);
  step("cmdline", record("cmdline", big, cmdline));
  step("dmesg", copy_dmesg());
  step("gadget", gadget());

  char file[128];
  long waited = 0;
  for (; waited < TIMEOUT_S; ++waited) {
    sleep_s(1);
    long n = read_file(LUN "/file", file, sizeof(file));
    if (n >= 0 && (n == 0 || file[0] == '\n')) break;   // the host ejected the medium
  }
  put(waited < TIMEOUT_S ? "ejected by the host" : "timeout"); put("; restarting into fastboot\n");
  flush_status();
  reboot_bootloader();
  for (;;) sys(SysExit, 1, 0, 0, 0, 0);
}
