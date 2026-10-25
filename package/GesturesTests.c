// Host test for GesturesCore.h. cc -std=c11 -Wall -Wextra -Werror package/GesturesTests.c && ./a.out
#include "GesturesCore.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(c) do{if(!(c)){printf("FAIL %s:%d %s\n",__FILE__,__LINE__,#c);exit(1);}}while(0)

int main(void) {
    // The corners of the panel in the finger set's units, and the middle of it (15240 of 30480, 10160 of 20320).
    CHECK(gesture_touch_x(0) == 0 && gesture_touch_y(0) == 0);
    CHECK(gesture_touch_x(GESTURE_PANEL_WIDTH - 1) < GESTURE_TOUCH_X_UNITS && gesture_touch_x(GESTURE_PANEL_WIDTH - 1) > 30400);
    CHECK(gesture_touch_y(GESTURE_PANEL_HEIGHT - 1) < GESTURE_TOUCH_Y_UNITS && gesture_touch_y(GESTURE_PANEL_HEIGHT - 1) > 20250);
    CHECK(gesture_touch_x(900) == 15240 && gesture_touch_y(1440) == 10160);

    // The unlock swipe starts inside NativeHidInput's edge band on y (or SpringBoard has no home gesture) and ends well inside
    // the panel; the page turns and the tap are ordinary contacts, not edge ones.
    const int unlockStart = gesture_touch_y(GESTURE_UNLOCK_START_Y), unlockEnd = gesture_touch_y(GESTURE_UNLOCK_END_Y);
    CHECK(gesture_near_y_edge(unlockStart) && !gesture_near_x_edge(gesture_touch_x(GESTURE_UNLOCK_X)));
    CHECK(unlockStart > unlockEnd + 3000);   // about half the panel travelled upward
    CHECK(!gesture_near_y_edge(unlockEnd) && !gesture_near_x_edge(gesture_touch_x(GESTURE_UNLOCK_X)));
    CHECK(!gesture_near_y_edge(gesture_touch_y(GESTURE_PAGE_Y)));
    CHECK(!gesture_near_x_edge(gesture_touch_x(GESTURE_PAGE_FROM_X)) && !gesture_near_x_edge(gesture_touch_x(GESTURE_PAGE_TO_X)));
    CHECK(!gesture_near_y_edge(gesture_touch_y(GESTURE_TAP_Y)));
    CHECK(gesture_touch_x(GESTURE_PAGE_FROM_X) > gesture_touch_x(GESTURE_PAGE_TO_X));   // right to left: the next page

    // A stroke's end points are exact, and its middle is between them.
    CHECK(gesture_lerp(GESTURE_UNLOCK_START_Y, GESTURE_UNLOCK_END_Y, 0, 30) == GESTURE_UNLOCK_START_Y);
    CHECK(gesture_lerp(GESTURE_UNLOCK_START_Y, GESTURE_UNLOCK_END_Y, 30, 30) == GESTURE_UNLOCK_END_Y);
    CHECK(gesture_lerp(GESTURE_PAGE_FROM_X, GESTURE_PAGE_TO_X, 30, 30) == GESTURE_PAGE_TO_X);
    const int middle = gesture_lerp(GESTURE_PAGE_FROM_X, GESTURE_PAGE_TO_X, 15, 30);
    CHECK(middle > GESTURE_PAGE_TO_X && middle < GESTURE_PAGE_FROM_X && middle == 900);
    // Every point stays inside the finger set's range, at every step count a stroke may use.
    for (int steps = 1; steps <= 60; ++steps) {
        for (int i = 0; i <= steps; ++i) {
            const int x = gesture_touch_x(gesture_lerp(GESTURE_PAGE_FROM_X, GESTURE_PAGE_TO_X, i, steps));
            const int y = gesture_touch_y(gesture_lerp(GESTURE_UNLOCK_START_Y, GESTURE_UNLOCK_END_Y, i, steps));
            CHECK(x >= 0 && x < GESTURE_TOUCH_X_UNITS && y >= 0 && y < GESTURE_TOUCH_Y_UNITS);
        }
    }
    puts("PASS: pipa scripted touches (geometry, edge band, stroke points)");
    return 0;
}
