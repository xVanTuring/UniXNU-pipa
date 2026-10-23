// Host test for ButtonsCore.h. cc -std=c11 -Wall -Wextra -Werror package/ButtonsTests.c && ./a.out
#include "ButtonsCore.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(c) do{if(!(c)){printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c);exit(1);}}while(0)

int main(void) {
    // Released: PON bits clear, GPIO6 high (pull-up).
    CHECK(buttons_mask(0, 1) == 0);
    CHECK(buttons_mask(BTN_PON_KPDPWR, 1) == BTN_POWER);
    CHECK(buttons_mask(BTN_PON_RESIN, 1) == BTN_VOLUME_DOWN);
    CHECK(buttons_mask(0, 0) == BTN_VOLUME_UP);
    CHECK(buttons_mask(BTN_PON_KPDPWR | BTN_PON_RESIN, 0) == (BTN_POWER | BTN_VOLUME_UP | BTN_VOLUME_DOWN));
    // Other PON status bits (bark, PMIC watchdog) and other GPIO bits are ignored.
    CHECK(buttons_mask(0xfc, 0xfe) == BTN_VOLUME_UP);
    // Inside NativeHidInput's 0xe.
    CHECK(((BTN_POWER | BTN_VOLUME_UP | BTN_VOLUME_DOWN) & ~0xeu) == 0);

    struct buttons_debounce d = {0, 0, 0};
    CHECK(!buttons_debounce_step(&d, 0, 2) && !buttons_debounce_step(&d, 0, 2) && d.stable == 0);
    // A single-sample glitch does not count.
    CHECK(!buttons_debounce_step(&d, BTN_VOLUME_UP, 2) && !buttons_debounce_step(&d, 0, 2) && d.stable == 0);
    // Two samples in a row do, once.
    CHECK(!buttons_debounce_step(&d, BTN_POWER, 2));
    CHECK(buttons_debounce_step(&d, BTN_POWER, 2) && d.stable == BTN_POWER);
    CHECK(!buttons_debounce_step(&d, BTN_POWER, 2));
    // Release.
    CHECK(!buttons_debounce_step(&d, 0, 2) && buttons_debounce_step(&d, 0, 2) && d.stable == 0);
    puts("PASS: pipa buttons (decoding and debounce)");
    return 0;
}
