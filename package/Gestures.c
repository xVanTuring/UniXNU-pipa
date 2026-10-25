// SPDX-License-Identifier: BSD-2-Clause-Patent
// Scripted touches for rounds nobody watches: `pipa-gesture=<seconds>` runs an unlock swipe up from the bottom edge, two page
// turns (right to left, then back) and a tap, starting that many seconds after the late probe (45 s at the earliest, 900 s at
// the most; 0 or no argument leaves it off, so the command line costs nothing unless a round asks for it).
//
// The events go in at the finger set NativeHidInput reports as its touch screen, the same interface the panel driver uses
// (TouchProbe.cpp publish), not at the driver above it: the producer makes native_touch_seq odd, writes count and the contacts,
// makes it even and calls native_touch_published, which reaches the sink the HID device registered in its start()
// (TouchState.cpp, NativeHidInput::touchNotify -> touchSend). From there the unchanged original IOHIDEventDriver turns the
// report into digitizer events, and NativeHidInput's dispatch wrapper adds the edge-tip bit that SpringBoard's home gesture
// needs. So the injection is indistinguishable from a finger on the panel except for the controller report that would precede
// it, and it needs no IOHIDDevice, no user client and no kext load of its own.
//
// The units are the panel's: the finger set runs x 0..30480 across the 1800-pixel width and y 0..20320 down the 2880-pixel
// height (the orientation the user's own strokes of 2026-10-24 confirmed). Rows go to the kernel log (BootRamoops.c) as they
// happen, with the HID reports and the presented display frames each step caused: the first says the event was sent, the second
// that the system acted on it. Nothing here can panic: no IOService, no allocation, no lock held across a sleep; a frame no
// consumer takes is simply counted by nobody.
#include <stdint.h>
#include "GesturesCore.h"

extern int PE_parse_boot_argn(const char *arg_string, void *arg_ptr, int max_arg);
extern int kernel_thread_start(void (*continuation)(void *, int), void *parameter, void **new_thread);
extern void thread_deallocate(void *thread);
extern void IOSleep(unsigned milliseconds);
extern int snprintf(char *, unsigned long, const char *, ...);

// The finger set (TouchState.cpp) and its consumer's notification.
extern volatile uint32_t native_touch_seq, native_touch_count, native_touch_hid_reports;
extern volatile int native_touch_xs[12], native_touch_ys[12], native_touch_ids[12];
extern void native_touch_published(void);
// Presented frames (LegacyDisplayProbe.cpp native_legacy_transfer_observed): the readiness gate and the per-step response.
extern volatile uint32_t native_legacy_transfer_calls;

#define FRAME_MS 10          // a report every 10 ms, the rate the panel's own reports arrive at
#define GESTURE_EARLIEST 45  // SpringBoard and its lock screen are up at about 50 s (board.toml)
#define GESTURE_LATEST 900
#define SETTLE_MS 40         // the work-loop action that sends a report runs shortly after the publish

// One row per step, printed once each by BootRamoops.c, and the count of rows written.
char native_gesture_rows[8][120];
volatile uint32_t native_gesture_written;
volatile int native_gesture_error;

static uint32_t elapsedMs;

static void wait_ms(uint32_t ms) {
    IOSleep(ms);
    elapsedMs += ms;
}
static char *row_slot(void) {return native_gesture_rows[native_gesture_written & 7];}
static void row_done(void) {native_gesture_written = native_gesture_written + 1;}

// The finger set, once: down at one contact.
static void finger(int x, int y) {
    native_touch_seq = native_touch_seq + 1;
    __asm__ volatile("dmb ish" ::: "memory");
    native_touch_count = 1;
    native_touch_xs[0] = x; native_touch_ys[0] = y; native_touch_ids[0] = 9;   // 9: the slot the controller's one-finger reports leave free
    __asm__ volatile("dmb ish" ::: "memory");
    native_touch_seq = native_touch_seq + 1;
    native_touch_published();
}
// No contact: NativeHidInput sends the tip-up for the identifier it saw down, at its last position.
static void lift(void) {
    native_touch_seq = native_touch_seq + 1;
    __asm__ volatile("dmb ish" ::: "memory");
    native_touch_count = 0;
    __asm__ volatile("dmb ish" ::: "memory");
    native_touch_seq = native_touch_seq + 1;
    native_touch_published();
}

// One stroke, and what it caused: the HID reports (the event reached the event driver) and the display frames (the UI reacted).
static void stroke(const char *name, int x0, int y0, int x1, int y1, unsigned ms) {
    const uint32_t hidBefore = native_touch_hid_reports, displayBefore = native_legacy_transfer_calls;
    const int steps = (int)(ms / FRAME_MS);
    for (int i = 0; i <= steps; ++i) {
        finger(gesture_touch_x(gesture_lerp(x0, x1, i, steps)), gesture_touch_y(gesture_lerp(y0, y1, i, steps)));
        wait_ms(FRAME_MS);
    }
    lift();
    wait_ms(SETTLE_MS);
    const uint32_t hid = native_touch_hid_reports - hidBefore, display = native_legacy_transfer_calls - displayBefore;
    snprintf(row_slot(), sizeof native_gesture_rows[0], "%s (%d,%d)->(%d,%d) %u ms, %d frames: hid +%u, display +%u", name, x0, y0,
             x1, y1, ms, steps + 1, hid, display);
    row_done();
    if (!hid) native_gesture_error = -3;   // nothing took the report: is sheng-touch-hid=1 and the event driver attached?
}

// A contact held in place, then released.
static void tap(int px, int py, unsigned ms) {
    const uint32_t hidBefore = native_touch_hid_reports, displayBefore = native_legacy_transfer_calls;
    const unsigned frames = ms / FRAME_MS;
    for (unsigned i = 0; i < frames; ++i) {finger(gesture_touch_x(px), gesture_touch_y(py)); wait_ms(FRAME_MS);}
    lift();
    wait_ms(SETTLE_MS);
    const uint32_t hid = native_touch_hid_reports - hidBefore, display = native_legacy_transfer_calls - displayBefore;
    snprintf(row_slot(), sizeof native_gesture_rows[0], "tap (%d,%d) %u ms, %u frames: hid +%u, display +%u", px, py, ms, frames, hid,
             display);
    row_done();
    if (!hid) native_gesture_error = -3;
}

static void gestures_thread(void *parameter, int result) {
    (void)result;
    const unsigned asked = (unsigned)(uintptr_t)parameter;
    unsigned start = asked;
    if (start < GESTURE_EARLIEST) start = GESTURE_EARLIEST;
    if (start > GESTURE_LATEST) start = GESTURE_LATEST;
    // Nothing is injected before the system draws: two presented frames is the same gate the HID input uses for its own
    // automatic Home press. The waits that give up are bounded, so a display that never comes up ends the thread quietly.
    while (native_legacy_transfer_calls < 2) {
        if (elapsedMs >= 180000) {
            native_gesture_error = -2;
            snprintf(row_slot(), sizeof native_gesture_rows[0], "stopped: no frame presented in 180 s");
            row_done();
            return;
        }
        wait_ms(200);
    }
    if (elapsedMs < start * 1000) wait_ms(start * 1000 - elapsedMs);
    const uint32_t hidStart = native_touch_hid_reports;
    snprintf(row_slot(), sizeof native_gesture_rows[0], "start at %u s (%u asked), %u display frames, %u hid reports", elapsedMs / 1000,
             asked, native_legacy_transfer_calls, hidStart);
    row_done();
    // Up from the bottom edge, where NativeHidInput marks the contact as an edge one, to the home screen; the user's own unlock
    // strokes of 2026-10-24 ran from (30480-wide) y 20200 to 15500 of 20320 in 250 to 360 ms.
    stroke("unlock", GESTURE_UNLOCK_X, GESTURE_UNLOCK_START_Y, GESTURE_UNLOCK_X, GESTURE_UNLOCK_END_Y, 300);
    // 10 s after the unlock: with 5 s the first page turn came too early on some rounds (frame buffers, 2026-10-24).
    wait_ms(10000);
    // Three page turns to the left reach the App Library, which is where the frame rate falls apart on the tablet (2026-10-25
    // evening: 9 to 15 frames a second while its blurred background is on screen, 40 and more elsewhere), then a dwell long enough
    // for the pacing window to cover it, then the same three turns back.
    for (unsigned turn = 0; turn < 3; ++turn) {
        stroke("page next", GESTURE_PAGE_FROM_X, GESTURE_PAGE_Y, GESTURE_PAGE_TO_X, GESTURE_PAGE_Y, 300);
        wait_ms(1500);
    }
    wait_ms(12000);
    for (unsigned turn = 0; turn < 3; ++turn) {
        stroke("page back", GESTURE_PAGE_TO_X, GESTURE_PAGE_Y, GESTURE_PAGE_FROM_X, GESTURE_PAGE_Y, 300);
        wait_ms(1500);
    }
    wait_ms(3000);
    tap(GESTURE_TAP_X, GESTURE_TAP_Y, 80);
    snprintf(row_slot(), sizeof native_gesture_rows[0], "done at %u s: hid +%u, %u display frames", elapsedMs / 1000,
             native_touch_hid_reports - hidStart, native_legacy_transfer_calls);
    row_done();
}

void native_gestures_probe(void) {
    static int started;
    unsigned start = 0;
    if (started || !PE_parse_boot_argn("pipa-gesture", &start, sizeof start) || !start) return;
    started = 1;
    void *thread = 0;
    if (kernel_thread_start(gestures_thread, (void *)(uintptr_t)start, &thread) == 0) thread_deallocate(thread);
    else native_gesture_error = -12;
}
