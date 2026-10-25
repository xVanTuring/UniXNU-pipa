// Host test for NativeBatteryBoard.h. cc -std=c11 -Wall -Wextra -Werror NativeBatteryTests.c && ./a.out
// The reference values are Android's own gauge output on this tablet (/sys/class/power_supply/bms at 9 % and the boot log's
// "battery SOC:100 voltage: 4484607uV temp: 365"), decoded with the formulas of Linux qpnp-fg-gen4.c / fg-util.c.
#include "NativeBatteryBoard.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(c) do{if(!(c)){printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c);exit(1);}}while(0)

int main(void) {
    // Endpoints and the 1..254 ramp of fg_get_msoc (255 = full, 0 = empty, (raw-1)*98/253+1 otherwise).
    CHECK(battery_percent(255) == 100);
    CHECK(battery_percent(254) == 99);
    CHECK(battery_percent(250) == 97);
    CHECK(battery_percent(128) == 50);
    CHECK(battery_percent(20) == 8);
    CHECK(battery_percent(1) == 1);
    CHECK(battery_percent(0) == 0);

    // VBATT: 4484607 uV on the tablet is 36739 counts of 122070/1000 uV.
    CHECK(battery_voltage_uv(36739) == 4484729);
    CHECK(battery_voltage_uv(0) == 0);
    CHECK(battery_voltage_uv(36303) == 4431507);
    // 4.4 V full-scale is 36045 counts; 3.0 V cutoff is about 24575.
    CHECK(battery_voltage_uv(36045) / 1000 == 4400);
    CHECK(battery_voltage_uv(24575) / 1000 == 2999);

    // IBATT: sign bit 15, 488281/1000 uA per count. The tablet's boot log had -68359 uA (charging) for -140 counts.
    CHECK(battery_current_ua(0xFF74) == -68359);
    CHECK(battery_current_ua(0x008C) == 68359);
    CHECK(battery_current_ua(0) == 0);
    CHECK(battery_current_ua(0x8000) == -15999991);
    CHECK(battery_current_ua(0x7FFF) == 15999503);

    // BATT_TEMP: 10-bit signed, 0.25 C. 365 deci-C is 146 counts; 35 C is 140 and UEFI's 37 C is 148.
    CHECK(battery_temp_millic(146) == 36500);
    CHECK(battery_temp_millic(140) == 35000);
    CHECK(battery_temp_millic(148) == 37000);
    // The field is bits 9:0: bit 10 and above are other data and must not turn 140 into a negative number.
    CHECK(battery_temp_millic(0x400 | 140) == 35000);
    CHECK(battery_temp_millic(0x8000 | 140) == 35000);
    CHECK(battery_temp_millic(0) == 0);
    CHECK(battery_temp_millic(0x3FF) == -250);
    CHECK(battery_temp_millic(0x200) == -128000);
    CHECK(battery_temp_millic(0x1FF) == 127750);

    // Charger stage (BATTERY_CHARGER_STATUS_1): only trickle..taper is charging; terminate, pause, inhibit and disable are not.
    CHECK(!battery_charging(0));
    CHECK(battery_charging(1));
    CHECK(battery_charging(2));
    CHECK(battery_charging(3));
    CHECK(battery_charging(4));
    CHECK(!battery_charging(5));
    CHECK(!battery_charging(6));
    CHECK(!battery_charging(7));

    // The gauge's copies are little-endian register pairs.
    const uint8_t sample[2] = {0x74, 0xFF};
    CHECK(battery_u16(sample) == 0xFF74);
    puts("PASS: pipa battery (SOC, voltage, current, temperature, charge stage)");
    return 0;
}
