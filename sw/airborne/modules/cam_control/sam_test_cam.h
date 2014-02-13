/*
 * Copyright (C) 2009-2012 Gautier Hattenberger <gautier.hattenberger@laas.fr>,
 *                    Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/cam_control/sam_test_cam.h
 * Camera control module for sam_test.
 *
 * The camera is controled by the heading of the vehicle for pan
 * and can be controlled by a servo for tilt if defined.
 *
 * Four modes:
 *  - NONE: no control
 *  - MANUAL: the servo position is set with PWM
 *  - HEADING: the servo position and the heading of the sam_test are set with angles
 *  - WP: the camera is tracking a waypoint (Default: CAM)
 *
 * The CAM_SWITCH can be used to power the camera in normal modes
 * and disable it when in NONE mode
 */

#ifndef SAM_TEST_CAM_H
#define SAM_TEST_CAM_H

#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "math/pprz_algebra_int.h"
#include "std.h"
#include "led.h"

#define SAM_TEST_CAM_MODE_NONE     0
#define SAM_TEST_CAM_MODE_MANUAL   1
#define SAM_TEST_CAM_MODE_HEADING  2
#define SAM_TEST_CAM_MODE_WP       3

/** Default mode is NONE. */
#ifndef SAM_TEST_CAM_DEFAULT_MODE
#define SAM_TEST_CAM_DEFAULT_MODE SAM_TEST_CAM_MODE_NONE
#endif

/** Cam power control.
 * By default CAM_SWITCH is used
 * Warning:
 *  LED_ON set GPIO low on some boards (lpc)
 *  LED_OFF set GPIO high on some boards (lpc)
 */
#ifndef SAM_TEST_CAM_ON
#ifdef CAM_SWITCH_LED
#define SAM_TEST_CAM_ON LED_OFF(CAM_SWITCH_LED)
#else
#define SAM_TEST_CAM_ON {}
#endif
#endif
#ifndef SAM_TEST_CAM_OFF
#ifdef CAM_SWITCH_LED
#define SAM_TEST_CAM_OFF LED_ON(CAM_SWITCH_LED)
#else
#define SAM_TEST_CAM_OFF {}
#endif
#endif

/** Cam tilt control.
 * By default use tilt control if a servo is assigned
 */
#ifdef SAM_TEST_CAM_TILT_SERVO
#define SAM_TEST_CAM_USE_TILT 1
#else
#define SAM_TEST_CAM_USE_TILT 0
#endif

/** Use angles for tilt in HEADING and WP modes.
 */
#if defined SAM_TEST_CAM_TILT_ANGLE_MIN && defined SAM_TEST_CAM_TILT_ANGLE_MAX && defined SAM_TEST_CAM_USE_TILT
#define CAM_TA_MIN ANGLE_BFP_OF_REAL(SAM_TEST_CAM_TILT_ANGLE_MIN)
#define CAM_TA_MAX ANGLE_BFP_OF_REAL(SAM_TEST_CAM_TILT_ANGLE_MAX)
#define SAM_TEST_CAM_USE_TILT_ANGLES 1
#endif

/** Cam pan control.
 * By default use pan control (heading)
 */
#ifndef SAM_TEST_CAM_USE_PAN
#define SAM_TEST_CAM_USE_PAN 1
#endif

/** WP control.
 * By default use WP_CAM waypoint if defined
 */
#ifndef SAM_TEST_CAM_TRACK_WP
#ifdef WP_CAM
#define SAM_TEST_CAM_TRACK_WP WP_CAM
#endif
#endif

extern uint8_t sam_test_cam_mode;

extern int16_t sam_test_cam_tilt;
extern int16_t sam_test_cam_pan;
extern int16_t sam_test_cam_tilt_pwm;

extern void sam_test_cam_init(void);
extern void sam_test_cam_periodic(void);

/** Set camera mode.
 * Camera is powered down in NONE mode if CAM_{ON|OFF} are defined
 */
#define sam_test_cam_SetCamMode(_v) { \
  sam_test_cam_mode = _v; \
  if (sam_test_cam_mode == SAM_TEST_CAM_MODE_NONE) { SAM_TEST_CAM_OFF; } \
  else { SAM_TEST_CAM_ON; } \
}

/** Cam control from datalink message.
 * camera tilt and pan are incremented by STICK_TILT_INC and STICK_PAN_INC
 * when maximum command is received from the stick
 */
#ifndef SAM_TEST_CAM_STICK_TILT_INC
#define SAM_TEST_CAM_STICK_TILT_INC RadOfDeg(10.)
#endif
#ifndef SAM_TEST_CAM_STICK_PAN_INC
#define SAM_TEST_CAM_STICK_PAN_INC RadOfDeg(20.)
#endif

#define SAM_TEST_CAM_STICK_PARSE(_dl_buffer) { \
  sam_test_cam_tilt += (int16_t)((ANGLE_BFP_OF_REAL(SAM_TEST_CAM_STICK_TILT_INC)/127.)*(float)DL_SAM_TEST_CAM_STICK_tilt(_dl_buffer)); \
  sam_test_cam_pan += (int16_t)((ANGLE_BFP_OF_REAL(SAM_TEST_CAM_STICK_PAN_INC)/127.)*(float)DL_SAM_TEST_CAM_STICK_pan(dl_buffer)); \
  INT32_COURSE_NORMALIZE(sam_test_cam_pan); \
}

#endif /* SAM_TEST_CAM_H */

