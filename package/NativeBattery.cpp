// SPDX-License-Identifier: BSD-2-Clause-Patent
// pipa's battery for iPadOS: a gen4 fuel-gauge and SMB5 charger client that publishes an IOPMPowerSource, the class
// powerd in the userland matches to build IOPSGetPowerSourcesInfo (SpringBoard's status bar reads that). Started by
// LegacyDisplayProbe.cpp's late probe, so the object exists before launchd starts powerd.
//
// Hardware: PM8150B (device tree qcom,pm8150b@2, SPMI USID 2). The gauge and charger registers are read through the same
// arbiter observer channels as package/Buttons.c. The one write path is the gauge's interleaved memory access, used only
// for the SRAM temperature word, through the EE 0 channel as package/BootRamoops.c writes the PON; "pipa-battery=2" keeps
// the driver to reads only (temperature from the batt-info registers) and "pipa-battery=0" disables it. Register addresses
// and conversions: Linux qpnp-fg-gen4.c, fg-reg.h, fg-memif.c, fg-util.c, smb5-lib.c (see NativeBatteryBoard.h).
//
// Everything is bounded: a gauge that does not answer leaves the object unpublished and is retried every 10 s, no register
// is configured, and the memory interface is released on every exit. Log row: "battery: <row>" in package/BootRamoops.c.
#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <IOKit/pwr_mgt/IOPMPowerSource.h>
#include <IOKit/pwr_mgt/IOPMPrivate.h>
#include <IOKit/pwr_mgt/RootDomain.h>
#include <libkern/c++/OSKext.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSSymbol.h>
#include <kern/thread.h>
#include <mach/kmod.h>
#include <pexpert/pexpert.h>
#include "NativeBatteryBoard.h"

extern "C" {
extern kmod_info_t g_kernel_kmod_info;
uint64_t ml_io_map(uint64_t phys_addr, uint64_t size);
void IOSleep(unsigned milliseconds);
int snprintf(char *, unsigned long, const char *, ...);
int PE_parse_boot_argn(const char *arg_string, void *arg_ptr, int max_arg);
void native_battery_static_init(void) __asm__("__GLOBAL__sub_I_NativeBattery.cpp");
}

#define SPMI_CORE 0x0C440000ull
#define SPMI_CHNLS 0x0C600000ull
#define SPMI_OBSRVR 0x0E600000ull
#define SPMI_CNFG 0x0C40A000ull
#define SPMI_APID_MAP 0x900u
#define SPMI_CNFG_OWNER 0x700u
#define SPMI_MAX_APIDS 512u
#define SPMI_OBS_APID_STRIDE 0x80ull
#define SPMI_CHNL_APID_STRIDE 0x10000ull
#define SPMI_CMD 0x00u
#define SPMI_STATUS 0x08u
#define SPMI_WDATA0 0x10u
#define SPMI_RDATA0 0x18u
#define SPMI_POLLS 100000u
#define BAT_POLL_MS 10000u
#define BAT_RDY_TRIES 100u
#define BAT_IMA_TRIES 8u

struct battery_sample {
    unsigned percent, stage, charge_enable, wd_cfg, soc_raw, temp_step;
    int voltage_mv, current_ma, temp_centi_c, temp_error;
    bool present, external, charging, full, temp_valid, temp_sram;
};

static battery_sample gSample;
static unsigned gMode = 1;
static unsigned gFailures;

char native_battery_row[120];
volatile unsigned native_battery_changes;
volatile int native_battery_error;
volatile unsigned native_battery_published;
volatile unsigned native_battery_match;

static volatile uint32_t *gCore, *gCnfg;
static uint32_t gApids;
static volatile uint32_t *gFgSoc, *gFgInfo, *gFgMemIf, *gChgr, *gBatIf, *gUsbIn, *gMisc, *gFgMemIfCh;

static volatile uint32_t *page_of(uint64_t phys) {
    uint64_t page = ml_io_map(phys & ~0xFFFull, 0x1000);
    return page ? (volatile uint32_t *)(uintptr_t)(page + (phys & 0xFFF)) : 0;
}

static int apid_of(uint32_t ppid, uint32_t *apid) {
    int found = -1;
    for (uint32_t i = 0; i < gApids; ++i) {
        uint32_t map = gCore[(SPMI_APID_MAP + 4 * i) / 4];
        if (!map || ((map >> 8) & 0xFFF) != ppid) continue;
        if (found < 0 || (gCnfg[(SPMI_CNFG_OWNER + 4 * i) / 4] & 7) == 0) found = (int)i;
    }
    if (found < 0) return -1;
    *apid = (uint32_t)found;
    return 0;
}

static int wait_done(volatile uint32_t *engine) {
    for (uint32_t poll = 0; poll < SPMI_POLLS; ++poll) {
        uint32_t status = engine[SPMI_STATUS / 4];
        if (status & 1) return (status & 0xE) ? 2 : 0;
    }
    return 1;
}

static int read8(volatile uint32_t *observer, uint8_t address, uint8_t *value) {
    if (!observer) return 3;
    observer[SPMI_CMD / 4] = (1u << 27) | ((uint32_t)address << 4);
    int rc = wait_done(observer);
    if (!rc) *value = (uint8_t)observer[SPMI_RDATA0 / 4];
    return rc;
}

static int write8(volatile uint32_t *channel, uint8_t address, uint8_t value) {
    if (!channel) return 3;
    channel[SPMI_WDATA0 / 4] = value;
    channel[SPMI_CMD / 4] = (uint32_t)address << 4;
    return wait_done(channel);
}

// Two bytes in one transaction, as the arbiter's regmap path issues the gauge's 16-bit SRAM address: WDATA0 is a 32-bit
// word holding payload bytes 0..3 (spmi-pmic-arb.c pmic_arb_write_data memcpy's bc+1 bytes into it), WDATA1 holds bytes
// 4..7, and the command's low three bits are the byte count minus one (pmic_arb_fmt_cmd_v2). Writing the high byte to
// WDATA1 instead of the second byte of WDATA0 sends address & 0xff alone with a two-byte count.
static int write16(volatile uint32_t *channel, uint8_t address, uint16_t value) {
    if (!channel) return 3;
    channel[SPMI_WDATA0 / 4] = (uint32_t)value;
    channel[SPMI_CMD / 4] = ((uint32_t)address << 4) | 1u;
    return wait_done(channel);
}

static int read16(volatile uint32_t *observer, uint8_t address, uint16_t *value) {
    uint8_t bytes[2] = {0, 0};
    int rc = read8(observer, address, &bytes[0]);
    if (!rc) rc = read8(observer, (uint8_t)(address + 1), &bytes[1]);
    if (!rc) *value = battery_u16(bytes);
    return rc;
}

static int iacs_ready(volatile uint32_t *memif) {
    for (unsigned i = 0; i < BAT_RDY_TRIES; ++i) {
        uint8_t status = 0;
        if (read8(memif, BAT_FG_IMA_OPR_STS, &status)) return -1;
        if (status & BAT_FG_IACS_RDY) return 0;
        IOSleep(5);
    }
    return -2;
}

// fg_interleaved_mem_read: claim the gauge's memory interface, read one non-burst word, release it on every path. The
// request bit reads one while the gauge algorithm holds the interface, so a busy gauge fails here instead of waiting.
static int fg_sram_read16(uint16_t word, uint16_t *value, unsigned *step) {
    uint8_t cfg = 0;
    *step = 1;
    if (read8(gFgMemIf, BAT_FG_MEM_INTF_CFG, &cfg)) return -1;
    for (unsigned i = 0; i < BAT_IMA_TRIES && (cfg & BAT_FG_MEM_GNT); ++i) {
        IOSleep(4);
        if (read8(gFgMemIf, BAT_FG_MEM_INTF_CFG, &cfg)) return -1;
    }
    if (cfg & BAT_FG_MEM_GNT) {*step = 2; return -2;}
    *step = 3;
    int rc = write8(gFgMemIfCh, BAT_FG_MEM_INTF_CFG, (uint8_t)(cfg | BAT_FG_MEM_GNT | BAT_FG_MEM_IACS_SLCT));
    uint8_t control = 0;
    if (!rc) rc = read8(gFgMemIf, BAT_FG_IMA_CTL, &control);
    if (!rc) rc = write8(gFgMemIfCh, BAT_FG_IMA_CTL, (uint8_t)(control & 0x3F));
    *step = 4;
    if (!rc) rc = iacs_ready(gFgMemIf);
    *step = 5;
    if (!rc) rc = write16(gFgMemIfCh, BAT_FG_ADDR_LSB, word);
    *step = 6;
    if (!rc) rc = iacs_ready(gFgMemIf);
    *step = 7;
    uint8_t bytes[2] = {0, 0};
    if (!rc) rc = read8(gFgMemIf, BAT_FG_SRAM_RD_DATA0, &bytes[0]);
    if (!rc) rc = read8(gFgMemIf, BAT_FG_SRAM_RD_DATA0 + 1, &bytes[1]);
    uint8_t release = 0;
    if (!read8(gFgMemIf, BAT_FG_MEM_INTF_CFG, &release)) {
        write8(gFgMemIfCh, BAT_FG_MEM_INTF_CFG, (uint8_t)(release & ~(BAT_FG_MEM_GNT | BAT_FG_MEM_IACS_SLCT)));
    }
    if (!rc) *value = battery_u16(bytes);
    return rc;
}

// The SRAM word as hexadecimal, once per distinct value: only a word whose decoded value is impossible is reported.
static void log_temp_raw(uint16_t raw, int centi) {
    static int last = -1;
    if ((int)raw == last) return;
    last = (int)raw;
    IOLog("NativeBattery: SRAM temperature word %u = 0x%04x (%d counts, %d centi-C) is outside the plausible range\n",
          BAT_FG_TEMP_SRAM_WORD, raw, (int)(int16_t)raw, centi);
}

static int sample_temp(battery_sample *s) {
    uint16_t first = 0, second = 0;
    int srma = -1;
    if (gMode == 1) {
        srma = fg_sram_read16(BAT_FG_TEMP_SRAM_WORD, &first, &s->temp_step);
        if (!srma) {
            unsigned step = 0;
            if (fg_sram_read16(BAT_FG_TEMP_SRAM_WORD, &second, &step) || first != second) srma = -3;
        }
        if (!srma) {
            int centi = battery_temp_millic(first) / 10;
            if (centi < -2000 || centi > 8000) {
                // A plausible pack is -20..80 C; anything else means we decoded the wrong word, not a cold battery.
                log_temp_raw(first, centi);
                srma = -7;
            } else {
                s->temp_centi_c = centi;
                s->temp_valid = true;
                s->temp_sram = true;
                return 0;
            }
        }
    }
    if (read16(gFgInfo, BAT_FG_BATT_TEMP_LSB, &first)) return srma < -1 ? srma : -4;
    // The batt-info temperature registers read back 0 on this gauge (Linux' gen4 driver keeps the value in SRAM), so a
    // zero count is no measurement rather than 0 C.
    if (!first) return srma < -1 ? srma : -6;
    int centi = battery_temp_millic(first) / 10;
    if (centi <= -4000 || centi >= 12500) return srma < -1 ? srma : -5;
    s->temp_centi_c = centi;
    s->temp_valid = true;
    s->temp_sram = false;
    return 0;
}

static int sample_now(battery_sample *s) {
    uint8_t soc[2] = {0, 0}, stage = 0, enable = 0, usbin = 0, batif = 0, wd = 0;
    uint16_t vbat = 0, vbat_cp = 0, ibat = 0, ibat_cp = 0;
    if (read8(gFgSoc, BAT_FG_MSOC, &soc[0]) || read8(gFgSoc, BAT_FG_MSOC_CP, &soc[1]) || soc[0] != soc[1]) return -1;
    if (read16(gFgInfo, BAT_FG_VBATT_LSB, &vbat) || read16(gFgInfo, BAT_FG_VBATT_LSB_CP, &vbat_cp) || vbat != vbat_cp) return -2;
    if (read16(gFgInfo, BAT_FG_IBATT_LSB, &ibat) || read16(gFgInfo, BAT_FG_IBATT_LSB_CP, &ibat_cp) || ibat != ibat_cp) return -3;
    if (read8(gChgr, BAT_SMB_CHG_STAGE, &stage) || read8(gChgr, BAT_SMB_CHG_ENABLE, &enable)) return -4;
    if (read8(gUsbIn, BAT_SMB_USBIN_RT_STS, &usbin) || read8(gBatIf, BAT_SMB_BATIF_RT_STS, &batif)) return -5;
    if (read8(gMisc, BAT_SMB_WDOG_CFG, &wd)) return -6;

    s->soc_raw = soc[0];
    s->percent = battery_percent(soc[0]);
    s->voltage_mv = battery_voltage_uv(vbat) / 1000;
    s->current_ma = -battery_current_ua(ibat) / 1000;
    s->stage = stage & 7u;
    s->charge_enable = enable & 1u;
    s->wd_cfg = wd;
    s->external = (usbin & BAT_SMB_USBIN_PLUGIN) != 0;
    s->present = (batif & BAT_SMB_BATT_MISSING) == 0;
    s->charging = battery_charging(s->stage) && s->external;
    s->full = s->stage == BAT_SMB_STAGE_TERMINATE;
    s->temp_valid = false;
    s->temp_sram = false;
    s->temp_step = 0;
    s->temp_error = sample_temp(s);
    return 0;
}

static void temp_text(int centi, char *text, unsigned capacity);

class NativeBattery : public IOPMPowerSource {
    OSDeclareDefaultStructors(NativeBattery)
    const OSSymbol *fFullyCharged = nullptr;
    const OSSymbol *fDesignCapacity = nullptr;
    const OSSymbol *fTemperature = nullptr;
    const OSSymbol *fTransport = nullptr;
    const OSSymbol *fType = nullptr;
    bool fPublished = false;
    bool fHasExternal = false;
    bool fExternal = false;
    void setNumber(const OSSymbol *key, long long value);
public:
    bool start(IOService *provider) override;
    void stop(IOService *provider) override;
    void update(const battery_sample &sample);
};

OSDefineMetaClassAndStructors(NativeBattery, IOPMPowerSource)

static NativeBattery *gBattery;

void NativeBattery::setNumber(const OSSymbol *key, long long value) {
    OSNumber *number = OSNumber::withNumber((unsigned long long)value, 32);
    if (!number) return;
    setPSProperty(key, number);
    number->release();
}

void NativeBattery::update(const battery_sample &s) {
    setBatteryInstalled(s.present);
    setExternalConnected(s.external);
    setExternalChargeCapable(s.external);
    setIsCharging(s.charging);
    setCurrentCapacity(s.present ? s.percent : 0);
    setMaxCapacity(100);
    setVoltage(s.voltage_mv > 0 ? (unsigned)s.voltage_mv : 0);
    setAmperage(s.current_ma);
    setAtWarnLevel(s.percent <= 20);
    setAtCriticalLevel(s.percent <= 5);
    unsigned minutes = BAT_TIME_UNKNOWN;
    int magnitude = s.current_ma < 0 ? -s.current_ma : s.current_ma;
    if (s.present && magnitude > 20) {
        unsigned long long milliamps = (unsigned long long)magnitude;
        unsigned long long capacity = (s.charging ? (100 - s.percent) : s.percent) * (unsigned long long)BAT_DESIGN_MAH / 100;
        unsigned long long estimate = capacity * 60 / milliamps;
        if (estimate < 0x7FFF) minutes = (unsigned)estimate;
    }
    setTimeRemaining((int)minutes);
    setNumber(fFullyCharged, s.full ? 1 : 0);
    setNumber(fDesignCapacity, BAT_DESIGN_MAH);
    if (s.temp_valid) setNumber(fTemperature, s.temp_centi_c > 0 ? s.temp_centi_c : 0);
    setPSProperty(fTransport, const_cast<OSSymbol *>(fTransport));
    setPSProperty(fType, const_cast<OSSymbol *>(fType));
    updateStatus();
    if (!fPublished) {
        fPublished = true;
        native_battery_published = 1;
        char temperature[12] = "";
        if (s.temp_valid) temp_text(s.temp_centi_c, temperature, sizeof temperature);
        IOLog("NativeBattery: published %u%% %d mV %d mA %s stage %u%s%s%s\n", s.percent, s.voltage_mv, s.current_ma,
              s.temp_valid ? temperature : "no temperature", s.stage, s.charging ? " charging" : "",
              s.external ? " external" : "", s.present ? "" : " no-battery");
    }
    if (!fHasExternal || fExternal != s.external) {
        IOPMrootDomain *domain = IOService::getPMRootDomain();
        if (domain) domain->receivePowerNotification(kIOPMSetACAdaptorConnected | (s.external ? kIOPMSetValue : 0));
        fHasExternal = true;
        fExternal = s.external;
    }
}

bool NativeBattery::start(IOService *provider) {
    if (!IOPMPowerSource::start(provider)) return false;
    fFullyCharged = OSSymbol::withCString("FullyCharged");
    fDesignCapacity = OSSymbol::withCString("DesignCapacity");
    fTemperature = OSSymbol::withCString("Temperature");
    fTransport = OSSymbol::withCString("Internal");
    fType = OSSymbol::withCString("InternalBattery");
    return fFullyCharged && fDesignCapacity && fTemperature && fTransport && fType;
}

void NativeBattery::stop(IOService *provider) {
    OSSafeReleaseNULL(fFullyCharged);
    OSSafeReleaseNULL(fDesignCapacity);
    OSSafeReleaseNULL(fTemperature);
    OSSafeReleaseNULL(fTransport);
    OSSafeReleaseNULL(fType);
    IOPMPowerSource::stop(provider);
}

static void temp_text(int centi, char *text, unsigned capacity) {
    int absolute = centi < 0 ? -centi : centi;
    snprintf(text, capacity, "%s%d.%02dC", centi < 0 ? "-" : "", absolute / 100, absolute % 100);
}

// One line per change of the reading; t: names the temperature source, or the SRAM step and error that stopped it.
static void publish_row(const battery_sample &s) {
    char temperature[12] = "";
    char source[20] = "";
    if (s.temp_valid) {
        temp_text(s.temp_centi_c, temperature, sizeof temperature);
        snprintf(source, sizeof source, "%s", s.temp_sram ? "sram" : "reg");
    } else {
        snprintf(source, sizeof source, "err(%u,%d)", s.temp_step, s.temp_error);
    }
    snprintf(native_battery_row, sizeof native_battery_row, "%u%%(r%u) %dmV %dmA %s stg%u en%u ext%u bat%u wd53=%02x t:%s%s",
             s.percent, s.soc_raw, s.voltage_mv, s.current_ma, s.temp_valid ? temperature : "n/a", s.stage, s.charge_enable,
             s.external, s.present, s.wd_cfg, source, s.charging ? " chg" : "");
}

// The interleaved-memory status registers, once per distinct failure: which step stopped the SRAM temperature read and
// what the gauge's memory interface reported at that moment.
static void log_temp_failure(const battery_sample &s) {
    static int last_error = 1, last_step = 1;
    if (s.temp_valid || (s.temp_error == last_error && (int)s.temp_step == last_step)) return;
    last_error = s.temp_error;
    last_step = (int)s.temp_step;
    uint8_t intf = 0, opr = 0, exp_sts = 0, hw = 0, err = 0;
    read8(gFgMemIf, BAT_FG_MEM_INTF_CFG, &intf);
    read8(gFgMemIf, BAT_FG_IMA_OPR_STS, &opr);
    read8(gFgMemIf, BAT_FG_IMA_EXP_STS, &exp_sts);
    read8(gFgMemIf, BAT_FG_IMA_HW_STS, &hw);
    read8(gFgMemIf, BAT_FG_IMA_ERR_STS, &err);
    IOLog("NativeBattery: SRAM temperature stopped at step %u error %d (intf %02x opr %02x exp %02x hw %02x err %02x)\n",
          s.temp_step, s.temp_error, intf, opr, exp_sts, hw, err);
}

// The class is registered once: preModLoad/postModLoad bracket the static initializers of a kmod's classes, and a second
// registration of the same metaclass is not allowed.
static int load_class(void) {
    static int state;
    if (state) return state > 0 ? 0 : state;
    void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if (!handle) {
        state = -1;
        return state;
    }
    native_battery_static_init();
    state = OSMetaClass::postModLoad(handle) ? -2 : 1;
    return state > 0 ? 0 : state;
}

// The path powerd uses: IOServiceMatching("IOPMPowerSource") only returns services whose kIOServiceMatchedState is set,
// which registerService() reaches at the end of the (normally asynchronous) matching job, not when it returns.
static unsigned verify_match(NativeBattery *battery) {
    OSDictionary *matching = IOService::serviceMatching("IOPMPowerSource");
    IOService *found = matching ? IOService::copyMatchingService(matching) : nullptr;
    unsigned result = found == battery ? 1u : (found ? 2u : 0u);
    if (found) found->release();
    OSSafeReleaseNULL(matching);
    return result;
}

static bool attach_battery(const battery_sample &sample) {
    if (load_class()) return false;
    NativeBattery *battery = new NativeBattery;
    if (!battery) return false;
    if (!battery->init()) {
        battery->release();
        return false;
    }
    IOService *domain = IOService::getPMRootDomain();
    if (!domain || !battery->attach(domain) || !battery->start(domain)) {
        battery->release();
        return false;
    }
    battery->setName("NativeBattery");
    battery->update(sample);
    battery->registerService(kIOServiceSynchronous);
    battery->retain();
    gBattery = battery;
    native_battery_match = verify_match(battery);
    IOLog("NativeBattery: IOPMPowerSource match %u\n", native_battery_match);
    return true;
}

// The first sample runs here, not in the late probe: nothing in that boot path may wait on the gauge or its memory
// interface. A sample that fails is retried on the next pass.
static void battery_thread(void *argument, int result) {
    (void)argument;
    (void)result;
    unsigned seconds = 0;
    bool publish_failure_logged = false;
    bool first = true;
    for (;;) {
        battery_sample sample = {};
        if (sample_now(&sample)) {
            native_battery_error = -1;
            if (!gFailures++) {
                snprintf(native_battery_row, sizeof native_battery_row, "no sample, the gauge did not answer");
                native_battery_changes = native_battery_changes + 1;
            }
        } else {
            bool changed = first || sample.percent != gSample.percent || sample.charging != gSample.charging ||
                           sample.external != gSample.external || sample.stage != gSample.stage ||
                           sample.voltage_mv != gSample.voltage_mv || sample.current_ma != gSample.current_ma ||
                           sample.temp_valid != gSample.temp_valid || sample.temp_centi_c != gSample.temp_centi_c ||
                           sample.temp_error != gSample.temp_error || sample.temp_step != gSample.temp_step;
            first = false;
            gSample = sample;
            gFailures = 0;
            native_battery_error = 0;
            log_temp_failure(sample);
            if (!gBattery && !attach_battery(sample)) {
                native_battery_error = -2;
                if (!publish_failure_logged) {
                    publish_failure_logged = true;
                    IOLog("NativeBattery: publish failed, retrying every %u s\n", BAT_POLL_MS / 1000);
                }
            } else if (gBattery) {
                gBattery->update(sample);
                if (!native_battery_match) {
                    unsigned found = verify_match(gBattery);
                    if (found) {
                        native_battery_match = found;
                        IOLog("NativeBattery: IOPMPowerSource match %u after %u s\n", found, seconds);
                    }
                }
                publish_row(sample);
                if (changed) native_battery_changes = native_battery_changes + 1;
            }
        }
        IOSleep(BAT_POLL_MS);
        seconds += BAT_POLL_MS / 1000;
    }
}

static int map_hardware(void) {
    gCore = (volatile uint32_t *)(uintptr_t)ml_io_map(SPMI_CORE, 0x2000);
    if (!gCore) return -1;
    gApids = gCore[1] & 0x7FF;
    if (gCore[0] < 0x50000000u || gCore[0] >= 0x70000000u || !gApids || gApids > SPMI_MAX_APIDS) return -2;
    gCnfg = (volatile uint32_t *)(uintptr_t)ml_io_map(SPMI_CNFG, 0x1000);
    if (!gCnfg) return -3;
    static const struct {uint32_t ppid; volatile uint32_t **observer;} map[] = {
        {BAT_PPID_FG_SOC, &gFgSoc}, {BAT_PPID_FG_INFO, &gFgInfo}, {BAT_PPID_FG_MEMIF, &gFgMemIf},
        {BAT_PPID_SMB_CHGR, &gChgr}, {BAT_PPID_SMB_BATIF, &gBatIf}, {BAT_PPID_SMB_USBIN, &gUsbIn},
        {BAT_PPID_SMB_MISC, &gMisc},
    };
    for (unsigned i = 0; i < sizeof map / sizeof map[0]; ++i) {
        uint32_t apid = 0;
        if (apid_of(map[i].ppid, &apid)) return -4;
        *map[i].observer = page_of(SPMI_OBSRVR + SPMI_OBS_APID_STRIDE * apid);
        if (!*map[i].observer) return -5;
    }
    if (gMode == 1) {
        uint32_t apid = 0;
        if (apid_of(BAT_PPID_FG_MEMIF, &apid)) return -6;
        gFgMemIfCh = page_of(SPMI_CHNLS + SPMI_CHNL_APID_STRIDE * apid);
        if (!gFgMemIfCh) return -7;
    }
    return 0;
}

extern "C" void native_battery_probe(void);
extern "C" void native_battery_probe(void) {
    static int started;
    if (started) return;
    unsigned mode = 1;
    if (PE_parse_boot_argn("pipa-battery", &mode, sizeof mode)) gMode = mode;
    if (!gMode || gMode > 2) return;
    started = 1;
    IOLog("NativeBattery: probe mode %u\n", gMode);
    int rc = map_hardware();
    if (rc) {
        native_battery_error = rc;
        IOLog("NativeBattery: SPMI map failed (%d)\n", rc);
        return;
    }
    thread_t thread = nullptr;
    if (kernel_thread_start(battery_thread, nullptr, &thread) == kIOReturnSuccess) thread_deallocate(thread);
    else native_battery_error = -12;
}
