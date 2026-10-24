/* SPDX-License-Identifier: BSD-2-Clause-Patent */
/* Host test for A650Gmu.c over a software register peer (GMU, RSCC and PDC blocks) that answers as a6xx_gmu.c expects:
 * the RSC acknowledges stop and start requests, the GMU reports its firmware ready once released and its HFI queues ready
 * once initialised, and answers each HFI command in the response queue (after UniXNU-sheng's ShengGmuHostTests.c). With a
 * path argument it loads that a650_gmu.bin; otherwise a synthetic image with ITCM, DTCM and icache blocks.
 * cc -std=c11 -Wall -Wextra -Werror -I <UniXNU>/kits/AdrenoFirmware package/A650Gmu.c package/A650GmuTests.c \
 *    <UniXNU>/kits/AdrenoFirmware/{AdrenoFirmware,AdrenoHfi,AdrenoHfiStartup,AdrenoHfiVotes}.c */
#include "A650Gmu.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);exit(1);}}while(0)

static const size_t bytes[A650GmuWindows] = {A650_GMU_CORE_BYTES, A650_GMU_RSCC_BYTES, A650_GMU_PDC_BYTES};
static struct {
    uint32_t *regs[A650GmuWindows];
    unsigned writes, hold_firmware, hold_hfi, hold_ack, hold_gx, fault_on_release, peer, commands, firmware_error_at;
    uint64_t time;
} f;
static A650GmuDevice device;
static A650GmuConfig config;
static uint32_t *core(uint32_t dword) {return f.regs[A650GmuCore] + dword;}
static uint32_t *rscc(uint32_t dword) {return f.regs[A650GmuRscc] + dword;}
static int read32(void *cookie, unsigned w, size_t at, uint32_t *out)
{CHECK(cookie == &f && w < A650GmuWindows && !(at & 3) && at + 4 <= bytes[w]); *out = f.regs[w][at / 4]; return 0;}
/* The GMU's side of the HFI: take the command at the queue's read index, answer it in the response queue. */
static void respond(void)
{
    volatile uint32_t *s = config.hfi.cpu;
    unsigned at = s[16];
    uint32_t header = s[1024 + at];
    static const unsigned ids[5] = {4, 3, 14, 10, 30}, words[5] = {59, 160, 2, 1, 4};
    CHECK(f.commands < 5 && (header & 255) == ids[f.commands] && ((header >> 8) & 255) == words[f.commands]);
    ++f.commands;
    s[16] = s[17];
    at = s[29];
    s[2048 + at] = (3u << 8) | 126u;
    s[2048 + ((at + 1) & 1023)] = header;
    s[2048 + ((at + 2) & 1023)] = f.commands == f.firmware_error_at ? 37 : 0;
    s[2048 + ((at + 3) & 1023)] = 0xfafafafa;
    s[29] = (at + 4) & 1023;
}
static int write32(void *cookie, unsigned w, size_t at, uint32_t value)
{
    CHECK(cookie == &f && w < A650GmuWindows && !(at & 3) && at + 4 <= bytes[w]);
    ++f.writes;
    const uint32_t dword = (uint32_t)(at / 4);
    f.regs[w][dword] = value;
    if (w != A650GmuCore) return 0;
    if (dword == 0x9307) {                                     /* RSCC_CONTROL_REQ */
        if (value & 1) *rscc(0x004) |= 1u << 16;               /* stopped: RSC_STATUS0_DRV0 bit 16 */
        if ((value & 2) && !f.hold_ack) *core(0x9308) |= 2;    /* started: RSCC_CONTROL_ACK bit 1 */
    }
    if (dword == 0x5000 && !value) {                           /* CM3_SYSRESET released */
        if (!f.hold_firmware) *core(0x501c) = *core(0x1c00 + 0xff8) <= 0x20010004u ? 0xbabefaceu : 0x100u;
        if (f.fault_on_release) *core(0x9305) = 1;             /* AO watchdog bite */
    }
    if (dword == 0x5186 && value && !f.hold_hfi) *core(0x5180) = 1;   /* HFI_CTRL_INIT -> HFI_CTRL_STATUS */
    if (dword == 0x5194 && value == 1 && f.peer) respond();           /* HOST2GMU_INTR_SET: a message */
    if (dword == 0x5194 && value == (1u << 30) && !f.hold_gx) {       /* HOST2GMU_INTR_SET: OOB GPU_SET */
        *core(0x5192) |= 1u << 31;                                    /* GMU2HOST_INTR_INFO: its acknowledgement */
        *core(0x50d0) &= ~0xc0u;                                      /* GX powered */
    }
    if (dword == 0x5191) *core(0x5192) &= ~value;                     /* GMU2HOST_INTR_CLR */
    return 0;
}
static int barrier(void *cookie) {CHECK(cookie == &f); return 0;}
static int now(void *cookie, uint64_t *value) {CHECK(cookie == &f); *value = f.time; return 0;}
static int wait_us(void *cookie, unsigned us) {CHECK(cookie == &f); f.time += us; return 0;}
static void *allocate(size_t n) {void *p = NULL; CHECK(!posix_memalign(&p, 4096, n)); memset(p, 0, n); return p;}
static void reset(void)
{
    for (unsigned i = 0; i < A650GmuWindows; ++i) memset(f.regs[i], 0, bytes[i]);
    f.writes = f.hold_firmware = f.hold_hfi = f.hold_ack = f.hold_gx = f.fault_on_release = f.peer = f.commands = f.firmware_error_at = 0;
    f.time = 1000000;
    *core(0x5000) = 1;                                          /* held in reset */
    *core(0x50d0) = 0xc0;                                       /* GX off, as read on the tablet */
    device = (A650GmuDevice){0};
    device.io = (A650GmuIo){&f, {A650_GMU_CORE_BYTES, A650_GMU_RSCC_BYTES, A650_GMU_PDC_BYTES}, read32, write32, barrier};
    device.services = (A650GmuServices){&f, now, wait_us};
}
static void put32(uint8_t *p, uint32_t v) {p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);}
static size_t block(uint8_t *at, uint32_t address, const uint32_t *words, unsigned n)
{
    put32(at, address); put32(at + 4, 4 * n); put32(at + 8, 0xefbeadde); put32(at + 12, 0xefbeadde);
    for (unsigned i = 0; i < n; ++i) put32(at + 16 + 4 * i, words[i]);
    return 16 + 4 * n;
}
static const uint8_t *load(const char *path, size_t *n)
{
    static uint8_t synthetic[512];
    if (path) {
        FILE *fp = fopen(path, "rb");
        CHECK(fp);
        fseek(fp, 0, SEEK_END); *n = (size_t)ftell(fp); fseek(fp, 0, SEEK_SET);
        uint8_t *data = malloc(*n); CHECK(data && fread(data, 1, *n, fp) == *n); fclose(fp);
        return data;
    }
    /* An ITCM block, an icache block, and DTCM blocks whose word at 0x10007fe0 is the version (newer than 0x20010004). */
    const uint32_t itcm[3] = {0x11, 0x22, 0x33}, icache[2] = {0x44, 0x55}, dtcm[2] = {0x66, 0x77}, version[2] = {0x88, 0x20020000};
    size_t at = 0;
    put32(synthetic, 0); put32(synthetic + 4, 0); put32(synthetic + 8, 1); put32(synthetic + 12, 0x200); at = 16;   /* metadata */
    at += block(synthetic + at, 0x400, itcm, 3);
    at += block(synthetic + at, 0x157000, icache, 2);
    at += block(synthetic + at, 0x10004000, dtcm, 2);
    at += block(synthetic + at, 0x10007fdc, version, 2);
    *n = at;
    return synthetic;
}
static void le16s(uint8_t *out, const unsigned *v, unsigned n) {for (unsigned i = 0; i < n; ++i) {out[2 * i] = (uint8_t)v[i]; out[2 * i + 1] = (uint8_t)(v[i] >> 8);}}

int main(int argc, char **argv)
{
    for (unsigned i = 0; i < A650GmuWindows; ++i) f.regs[i] = allocate(bytes[i]);
    size_t firmware_bytes;
    const uint8_t *firmware = load(argc > 1 ? argv[1] : NULL, &firmware_bytes);
    config = (A650GmuConfig){A650_GMU_CHIP_ID, firmware, firmware_bytes,
        {allocate(0xffc000), 0xffc000, 0x4000}, {allocate(0x4000), 0x4000, 0x60000000},
        {allocate(0x4000), 0x4000, 0x60004000}, {allocate(0x4000), 0x4000, 0x60008000}, 8};
    const uint32_t chip_gmu = 0x06050200;   /* 0x06050002 with minor 0 and patch 2 in bits 15:12 and 11:8 */

    /* The pipa command DB's level lists (2026-10-24) and the HFI tables built from them. */
    uint8_t gfx[32] = {0}, mx[32] = {0}, cx[32] = {0};
    le16s(gfx, (const unsigned[]){0, 48, 64, 128, 144, 192, 224, 256, 320, 384, 416}, 11);
    le16s(mx, (const unsigned[]){0, 16, 128, 192, 224, 256, 320, 384, 416}, 9);
    le16s(cx, (const unsigned[]){0, 16, 64, 128, 192, 256, 320, 384, 416}, 9);
    AdrenoHfiStartupConfig startup;
    CHECK(A650GmuBuildStartup(gfx, 32, mx, 32, cx, 32, 0x50000, 0x50004, 0x5007c, 0x50074, &startup) == AdrenoHfiStartupSuccess);
    CHECK(startup.gpu_count == 8 && startup.gmu_count == 2 && startup.gpu[7].hz == 670000000 && startup.gmu[1].hz == 200000000);
    CHECK(startup.gpu[0].vote != startup.gpu[1].vote && startup.ddr_addresses[2] == 0x5007c && startup.cnoc_addresses[0] == 0x50074);
    CHECK(startup.cnoc_data[1][0] == 0x60000001 && startup.bandwidth_count == 1);
    printf("votes: gpu");
    for (unsigned i = 0; i < 8; ++i) printf(" %08x", startup.gpu[i].vote);
    printf(", gmu %08x %08x\n", startup.gmu[0].vote, startup.gmu[1].vote);
    CHECK(A650GmuBuildStartup(gfx, 32, mx, 32, cx, 32, 0, 0x50004, 0x5007c, 0x50074, &startup) == AdrenoHfiStartupInvalid);
    CHECK(A650GmuBuildStartup(gfx, 32, mx, 32, cx, 32, 0x50000, 0x50004, 0x5007c, 0x50074, &startup) == AdrenoHfiStartupSuccess);
    const A650GmuStartupLimits limits = {100, 100, 50000, 2000000};

    /* The whole cold start. */
    {
        reset(); f.peer = 1;
        A650GmuStartupResult r;
        CHECK(A650GmuColdStart(&device, &config, &startup, &limits, &r) == A650GmuSuccess);
        CHECK(r.load.phase == 8 && r.load.firmware_ready_observed && r.load.hfi_ready_observed && f.commands == 4);
        CHECK(r.protocol_status == AdrenoHfiStartupSuccess && r.protocol.published_commands == 4);
        /* The RSC and PDC setup, then the boot registers as a6xx_gmu_fw_start writes them. */
        CHECK(*rscc(0x180) == 0xeaaae5a0u && *rscc(0x184) == 0x0020edadu && *rscc(0x008) == 0x4520);
        CHECK(f.regs[A650GmuPdc][0x1140] == 0x80000001u && f.regs[A650GmuPdc][0x1148] == 0);
        CHECK(*core(0x51cc) == 1 && *core(0x50f0) == 1 && *core(0x50f1) == 1 && *core(0x9307) == 0);
        CHECK(*core(0x5185) == 0x60008000 && *core(0x5184) == 1 && *core(0x5001) == 2 && *core(0x502d) == 0x4052);
        CHECK(*core(0x5182) == chip_gmu && *core(0x5100) == (0x60004000 | 3) && *core(0x9311) == ((1u << 31) | (0xau << 18) | 0xa0));
        CHECK(*core(0x4c0f) == 1 && *core(0x50c0) == 0x9c40400u && (*core(0x50e8) & 0xf11) == 0xf11);
        CHECK(*core(0x5000) == 0 && *core(0x5193) == ~0x00800000u && *core(0x9306) == ~0x29u);
        /* The firmware words where a6xx_gmu_fw_load puts them. */
        if (argc <= 1) {
            CHECK(*core(0x0c00 + 0x100) == 0x11 && *core(0x0c00 + 0x102) == 0x33);
            CHECK(config.icache.cpu[(0x157000 - 0x4000) / 4] == 0x44 && config.icache.cpu[(0x157000 - 0x4000) / 4 + 1] == 0x55);
            CHECK(*core(0x1c00) == 0x66 && *core(0x1c00 + 0xff8) == 0x20020000 && r.load.firmware_version == 0x20020000);
            CHECK(r.load.firmware_init_result == 0x100);
        }
        printf("cold start: %u blocks, version %08x, ready %08x, %u register writes\n", r.load.blocks, r.load.firmware_version,
               r.load.firmware_init_result, f.writes);
        /* One attempt per device. */
        CHECK(A650GmuColdStart(&device, &config, &startup, &limits, &r) == A650GmuBusy);
        /* GX: the perf vote (fifth message), then the GPU_SET request and its acknowledgement, cleared afterwards. */
        A650GmuGxResult gx;
        CHECK(A650GmuPowerGx(&device, 8, &gx) == A650GmuArgument);
        CHECK(A650GmuPowerGx(&device, 1, &gx) == A650GmuSuccess && gx.gx_on && f.commands == 5);
        CHECK(gx.vote_status == AdrenoHfiSuccess && gx.gx_before == 0xc0 && gx.gx_after == 0 && gx.oob_polls == 1);
        CHECK(!(*core(0x5192) & (1u << 31)));
        printf("gx: vote %u, ack after %u polls, status %08x > %08x\n", gx.vote_status, gx.oob_polls, gx.gx_before, gx.gx_after);
    }
    /* GX without a completed cold start is refused; a GPU_SET that is never acknowledged times out. */
    {
        reset();
        A650GmuGxResult gx;
        CHECK(A650GmuPowerGx(&device, 1, &gx) == A650GmuStateMismatch && f.writes == 0);
        reset(); f.peer = 1; f.hold_gx = 1;
        A650GmuStartupResult r;
        CHECK(A650GmuColdStart(&device, &config, &startup, &limits, &r) == A650GmuSuccess);
        CHECK(A650GmuPowerGx(&device, 1, &gx) == A650GmuTimeout && !gx.gx_on && gx.oob_polls == 101 && gx.gx_after == 0xc0);
    }
    /* A GMU not held in reset is refused before any write. */
    {
        reset(); *core(0x5000) = 0;
        A650GmuResult r;
        CHECK(A650GmuLoadAndStartHfi(&device, &config, &r) == A650GmuBusy && f.writes == 0);
    }
    /* A busy RSC sequencer or a stale HFI ready bit too. */
    {
        reset(); *rscc(0x101) = 1;
        A650GmuResult r;
        CHECK(A650GmuLoadAndStartHfi(&device, &config, &r) == A650GmuBusy && f.writes == 0);
        reset(); *core(0x5180) = 1;
        CHECK(A650GmuLoadAndStartHfi(&device, &config, &r) == A650GmuStateMismatch && f.writes == 0);
    }
    /* No RSC acknowledgement: a timeout in phase 3, the GMU back in reset, the device poisoned. */
    {
        reset(); f.hold_ack = 1;
        A650GmuResult r;
        CHECK(A650GmuLoadAndStartHfi(&device, &config, &r) == A650GmuTimeout && r.phase == 3 && r.reset_verified);
        CHECK(*core(0x5000) == 1 && device.poisoned);
        CHECK(A650GmuLoadAndStartHfi(&device, &config, &r) == A650GmuPoisoned);
    }
    /* Firmware that never reports ready, an HFI that never starts, a watchdog bite after release. */
    {
        A650GmuResult r;
        reset(); f.hold_firmware = 1;
        CHECK(A650GmuLoadAndStartHfi(&device, &config, &r) == A650GmuTimeout && r.phase == 6 && *core(0x5000) == 1);
        reset(); f.hold_hfi = 1;
        CHECK(A650GmuLoadAndStartHfi(&device, &config, &r) == A650GmuTimeout && r.phase == 7 && r.firmware_ready_observed);
        reset(); f.fault_on_release = 1;
        CHECK(A650GmuLoadAndStartHfi(&device, &config, &r) == A650GmuFault && r.phase == 6 && r.ao_fault_status == 1);
    }
    /* A firmware error on the bandwidth table: the protocol stops there and the GMU goes back to reset. */
    {
        reset(); f.peer = 1; f.firmware_error_at = 2;
        A650GmuStartupResult r;
        CHECK(A650GmuColdStart(&device, &config, &startup, &limits, &r) == A650GmuProtocolError);
        CHECK(r.protocol.stage == AdrenoHfiStageBandwidth && r.protocol.firmware_error == 37 && *core(0x5000) == 1);
    }
    /* Configuration refusals. */
    {
        A650GmuResult r;
        reset();
        A650GmuConfig bad = config; bad.chip_id = 0x43050a01;
        CHECK(A650GmuLoadAndStartHfi(&device, &bad, &r) == A650GmuArgument);
        bad = config; bad.dummy.iova = 0x60400000;
        CHECK(A650GmuLoadAndStartHfi(&device, &bad, &r) == A650GmuArgument);
        bad = config; bad.log.iova = bad.hfi.iova;
        CHECK(A650GmuLoadAndStartHfi(&device, &bad, &r) == A650GmuArgument);
        CHECK(f.writes == 0);
    }
    puts("PASS: A650 GMU cold start (RSC/PDC setup, firmware load, boot registers, release, HFI start and messages), "
         "3 pre-write refusals, 4 failures that reset the GMU, 3 configuration refusals");
    return 0;
}
