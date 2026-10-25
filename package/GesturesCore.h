// SPDX-License-Identifier: BSD-2-Clause-Patent
// The geometry of the scripted touches in Gestures.c, for the host test GesturesTests.c.
//
// The panel is 1800 x 2880 pixels; the finger set NativeHidInput reports runs x 0..30480 across the width and y 0..20320 down
// the height (TouchProbe.cpp's units, the orientation the user's own strokes of 2026-10-24 confirmed: a swipe up is a falling
// y). Every point of the script is written in panel pixels and converted here, so the log rows can be read against a
// screenshot.
//
// NativeHidInput marks a contact that starts within 48 of its reference panel's 3048 x 2032 pixels as an edge one
// (kIOHIDDigitizerEventFromEdgeTip, NativeHidInput.cpp edge::kEdgeX/kEdgeY), which is 480 of the finger set's units on
// either axis; SpringBoard's home gesture needs that bit, so the unlock swipe must start inside the band.
#ifndef PIPA_GESTURES_CORE_H
#define PIPA_GESTURES_CORE_H
#include <stdint.h>

#define GESTURE_PANEL_WIDTH 1800
#define GESTURE_PANEL_HEIGHT 2880
#define GESTURE_TOUCH_X_UNITS 30480
#define GESTURE_TOUCH_Y_UNITS 20320
#define GESTURE_EDGE_UNITS 480   // 48 of 3048 x 2032, in the finger set's units

// The unlock swipe: up from the last rows of the panel, where the contact counts as an edge one, to the home screen. The user's
// own unlock strokes ran from y 20200 of 20320 to 15500 in 250 to 360 ms.
#define GESTURE_UNLOCK_X 900
#define GESTURE_UNLOCK_START_Y 2870
#define GESTURE_UNLOCK_END_Y 1300
// The page turns across the middle of the panel, the reach that turned the pages on the tablet (w/5 to w*4/5 at h/2).
#define GESTURE_PAGE_Y 1440
#define GESTURE_PAGE_FROM_X 1440
#define GESTURE_PAGE_TO_X 360
// The tap after them.
#define GESTURE_TAP_X 900
#define GESTURE_TAP_Y 2000
// The App Library's list, scrolled through its middle (script 1).
#define GESTURE_PAGE_X_MIDDLE 900
#define GESTURE_SCROLL_LOW_Y 2200
#define GESTURE_SCROLL_HIGH_Y 1000
// The dock's five icons (script 2), from the frame buffer pictures of 2026-10-25 (fbpic row 34, 40 x 80 pixel cells): icons in columns
// 13-15, 17-19, 21-23, 25-27 and 29-31 at y 2760, the dock platter alone at y 2680, nothing at y 2840. Above the edge band (2812).
#define GESTURE_DOCK_Y 2760
#define GESTURE_DOCK_X1 580
#define GESTURE_DOCK_X2 740
#define GESTURE_DOCK_X3 900
#define GESTURE_DOCK_X4 1060
#define GESTURE_DOCK_X5 1220

static inline int gesture_touch_x(int px) {return px * GESTURE_TOUCH_X_UNITS / GESTURE_PANEL_WIDTH;}
static inline int gesture_touch_y(int py) {return py * GESTURE_TOUCH_Y_UNITS / GESTURE_PANEL_HEIGHT;}
// With the point after it, so the band's inner boundary does not matter.
static inline int gesture_near_x_edge(int x) {return x < GESTURE_EDGE_UNITS || x > GESTURE_TOUCH_X_UNITS - GESTURE_EDGE_UNITS;}
static inline int gesture_near_y_edge(int y) {return y < GESTURE_EDGE_UNITS || y > GESTURE_TOUCH_Y_UNITS - GESTURE_EDGE_UNITS;}
// Point `i` of the `steps + 1` a stroke is sent in; the last one lands exactly on the end, so the release is where the script
// says and not one interpolation short of it.
static inline int gesture_lerp(int from, int to, int i, int steps) {return from + (to - from) * i / steps;}

#endif
