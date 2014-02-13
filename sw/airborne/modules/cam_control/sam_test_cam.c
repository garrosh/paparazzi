/*
 * Copyright (C) 2009-2012 Gautier Hattenberger <gautier.hattenberger@enac.fr>,
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
 * @file modules/cam_control/sam_test_cam.c
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

#include "modules/cam_control/sam_test_cam.h"

#include "subsystems/actuators.h"
#include "state.h"
#include "firmwares/sam_test/navigation.h"
#include "std.h"

#include "subsystems/datalink/telemetry.h"

uint8_t sam_test_cam_mode;

#define _SERVO_PARAM(_s,_p) SERVO_ ## _s ## _ ## _p
#define SERVO_PARAM(_s,_p) _SERVO_PARAM(_s,_p)

// Tilt definition
int16_t sam_test_cam_tilt;
int16_t sam_test_cam_tilt_pwm;
#if SAM_TEST_CAM_USE_TILT
#define SAM_TEST_CAM_TILT_NEUTRAL SERVO_PARAM(SAM_TEST_CAM_TILT_SERVO, NEUTRAL)
#define SAM_TEST_CAM_TILT_MIN SERVO_PARAM(SAM_TEST_CAM_TILT_SERVO, MIN)
#define SAM_TEST_CAM_TILT_MAX SERVO_PARAM(SAM_TEST_CAM_TILT_SERVO, MAX)
#define D_TILT (SAM_TEST_CAM_TILT_MAX - SAM_TEST_CAM_TILT_MIN)
#define CT_MIN Min(CAM_TA_MIN, CAM_TA_MAX)
#define CT_MAX Max(CAM_TA_MIN, CAM_TA_MAX)
#endif

// Pan definition
int16_t sam_test_cam_pan;
#define SAM_TEST_CAM_PAN_MIN 0
#define SAM_TEST_CAM_PAN_MAX INT32_ANGLE_2_PI

static void send_cam(void) {
  DOWNLINK_SEND_SAM_TEST_CAM(DefaultChannel, DefaultDevice,
      &sam_test_cam_tilt,&sam_test_cam_pan);
}

void sam_test_cam_init(void) {
  sam_test_cam_SetCamMode(SAM_TEST_CAM_DEFAULT_MODE);
#if SAM_TEST_CAM_USE_TILT
  sam_test_cam_tilt_pwm = SAM_TEST_CAM_TILT_NEUTRAL;
  ActuatorSet(SAM_TEST_CAM_TILT_SERVO, sam_test_cam_tilt_pwm);
#else
  sam_test_cam_tilt_pwm = 1500;
#endif
  sam_test_cam_tilt = 0;
  sam_test_cam_pan = 0;

  register_periodic_telemetry(DefaultPeriodic, "SAM_TEST_CAM", send_cam);
}

void sam_test_cam_periodic(void) {

  switch (sam_test_cam_mode) {
    case SAM_TEST_CAM_MODE_NONE:
#if SAM_TEST_CAM_USE_TILT
      sam_test_cam_tilt_pwm = SAM_TEST_CAM_TILT_NEUTRAL;
#endif
#if SAM_TEST_CAM_USE_PAN
      sam_test_cam_pan = stateGetNedToBodyEulers_i()->psi;
#endif
      break;
    case SAM_TEST_CAM_MODE_MANUAL:
      // nothing to do here, just apply tilt pwm at the end
      break;
    case SAM_TEST_CAM_MODE_HEADING:
#if SAM_TEST_CAM_USE_TILT_ANGLES
      Bound(sam_test_cam_tilt,CT_MIN,CT_MAX);
      sam_test_cam_tilt_pwm = SAM_TEST_CAM_TILT_MIN + D_TILT * (sam_test_cam_tilt - CAM_TA_MIN) / (CAM_TA_MAX - CAM_TA_MIN);
#endif
#if SAM_TEST_CAM_USE_PAN
      INT32_COURSE_NORMALIZE(sam_test_cam_pan);
      nav_heading = sam_test_cam_pan;
#endif
      break;
    case SAM_TEST_CAM_MODE_WP:
#ifdef SAM_TEST_CAM_TRACK_WP
      {
        struct Int32Vect2 diff;
        VECT2_DIFF(diff, waypoints[SAM_TEST_CAM_TRACK_WP], *stateGetPositionEnu_i());
        INT32_VECT2_RSHIFT(diff,diff,INT32_POS_FRAC);
        INT32_ATAN2(sam_test_cam_pan,diff.x,diff.y);
        nav_heading = sam_test_cam_pan;
#if SAM_TEST_CAM_USE_TILT_ANGLES
        int32_t dist, height;
        INT32_VECT2_NORM(dist, diff);
        height = (waypoints[SAM_TEST_CAM_TRACK_WP].z - stateGetPositionEnu_i()->z) >> INT32_POS_FRAC;
        INT32_ATAN2(sam_test_cam_tilt, height, dist);
        Bound(sam_test_cam_tilt, CAM_TA_MIN, CAM_TA_MAX);
        sam_test_cam_tilt_pwm = SAM_TEST_CAM_TILT_MIN + D_TILT * (sam_test_cam_tilt - CAM_TA_MIN) / (CAM_TA_MAX - CAM_TA_MIN);
#endif
      }
#endif
      break;
    default:
      break;
  }
#if SAM_TEST_CAM_USE_TILT
  ActuatorSet(SAM_TEST_CAM_TILT_SERVO, sam_test_cam_tilt_pwm);
#endif
}

