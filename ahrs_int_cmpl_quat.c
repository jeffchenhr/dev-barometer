/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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
 * @file subsystems/ahrs/ahrs_int_cmpl_quat.c
 *
 * Quaternion complementary filter (fixed-point).
 *
 * Estimate the attitude, heading and gyro bias.
 *
 */

#include "ahrs_int_cmpl_quat.h"
#include "ahrs_aligner.h"
#include "ahrs_int_utils.h"

//#include "state.h"

#include "imu.h"
#if USE_GPS
#include "subsystems/gps.h"
#endif
#include "math/pprz_trig_int.h"
#include "math/pprz_algebra_int.h"

#include "airframe.h"

int32_t gyro_lowpass_filter = 10;

  /**
   * Angular rates in body frame.
   * Units: rad/s in BFP with #INT32_RATE_FRAC
   */
  struct Int32Rates  body_rates_i;

  /**
   * Orientation quaternion.
   * Units: #INT32_QUAT_FRAC
   */
  struct Int32Quat ned_to_body_orientation_quat_i;

  struct Int32Quat ned_to_body_orientation_goal_quat_i;

  struct Int32Quat vicon_quat_i;
  struct Int32Vect3 vicon_pos_i;
  struct Int32Vect3 vicon_vel_i;
  struct Int32Vect3 vicon_acc_i;


  /**
   * Orientation rotation matrix.
   * Units: rad in BFP with #INT32_TRIG_FRAC
   */
  struct Int32RMat ned_to_body_orientation_rmat_i;

//#include "../../test/pprz_algebra_print.h"

static inline void ahrs_update_mag_full(void);
static inline void ahrs_update_mag_2d(void);

#ifdef AHRS_MAG_UPDATE_YAW_ONLY
#warning "AHRS_MAG_UPDATE_YAW_ONLY is deprecated, please remove it. This is the default behaviour. Define AHRS_MAG_UPDATE_ALL_AXES to use mag for all axes and not only yaw."
#endif

#if USE_MAGNETOMETER && AHRS_USE_GPS_HEADING
#warning "Using magnetometer and GPS course to update heading. Probably better to set USE_MAGNETOMETER=0 if you want to use GPS course."
#endif

#ifndef AHRS_PROPAGATE_FREQUENCY
#define AHRS_PROPAGATE_FREQUENCY PERIODIC_FREQUENCY
#endif

struct AhrsIntCmpl ahrs_impl;

#ifdef AHRS_UPDATE_FW_ESTIMATOR
// remotely settable
#ifndef INS_ROLL_NEUTRAL_DEFAULT
#define INS_ROLL_NEUTRAL_DEFAULT 0
#endif
#ifndef INS_PITCH_NEUTRAL_DEFAULT
#define INS_PITCH_NEUTRAL_DEFAULT 0
#endif
float ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
float ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
#endif

static inline void set_body_state_from_quat(void);


void ahrs_init(void) {

  QUAT_ASSIGN(ned_to_body_orientation_quat_i, 1, 0, 0, 0);
  INT32_QUAT_NORMALIZE(ned_to_body_orientation_quat_i);
  RATES_ASSIGN(body_rates_i, 0, 0, 0);


  ahrs.status = AHRS_UNINIT;
  ahrs_impl.ltp_vel_norm_valid = FALSE;
  ahrs_impl.heading_aligned = FALSE;

  /* set ltp_to_imu so that body is zero */
  QUAT_COPY(ahrs_impl.ltp_to_imu_quat, imu.body_to_imu_quat);
  INT_RATES_ZERO(ahrs_impl.imu_rate);

  INT_RATES_ZERO(ahrs_impl.gyro_bias);
  INT_RATES_ZERO(ahrs_impl.rate_correction);
  INT_RATES_ZERO(ahrs_impl.high_rez_bias);

#if AHRS_GRAVITY_UPDATE_COORDINATED_TURN
  ahrs_impl.correct_gravity = TRUE;
#else
  ahrs_impl.correct_gravity = FALSE;
#endif

#if AHRS_GRAVITY_UPDATE_NORM_HEURISTIC
  ahrs_impl.use_gravity_heuristic = TRUE;
#else
  ahrs_impl.use_gravity_heuristic = FALSE;
#endif

  VECT3_ASSIGN(ahrs_impl.mag_h, MAG_BFP_OF_REAL(AHRS_H_X), MAG_BFP_OF_REAL(AHRS_H_Y), MAG_BFP_OF_REAL(AHRS_H_Z));

}

void ahrs_align(void) {

#if USE_MAGNETOMETER
  /* Compute an initial orientation from accel and mag directly as quaternion */
  ahrs_int_get_quat_from_accel_mag(&ahrs_impl.ltp_to_imu_quat, &ahrs_aligner.lp_accel, &ahrs_aligner.lp_mag);
  ahrs_impl.heading_aligned = TRUE;
#else
  /* Compute an initial orientation from accel and just set heading to zero */
  ahrs_int_get_quat_from_accel(&ahrs_impl.ltp_to_imu_quat, &ahrs_aligner.lp_accel);
  ahrs_impl.heading_aligned = FALSE;
#endif

  set_body_state_from_quat();

  /* Use low passed gyro value as initial bias */
  RATES_COPY( ahrs_impl.gyro_bias, ahrs_aligner.lp_gyro);
  RATES_COPY( ahrs_impl.high_rez_bias, ahrs_aligner.lp_gyro);
  INT_RATES_LSHIFT(ahrs_impl.high_rez_bias, ahrs_impl.high_rez_bias, 28);

  ahrs.status = AHRS_RUNNING;
}



/*
 *
 *
 *
 */
void ahrs_propagate(void) {

  /* unbias gyro             */
  struct Int32Rates omega;
  RATES_DIFF(omega, imu.gyro_prev, ahrs_impl.gyro_bias);

  /* low pass rate */
//#ifdef AHRS_PROPAGATE_LOW_PASS_RATES
  if (gyro_lowpass_filter > 1) {
	  RATES_SMUL(ahrs_impl.imu_rate, ahrs_impl.imu_rate, gyro_lowpass_filter-1);
	  RATES_ADD(ahrs_impl.imu_rate, omega);
	  RATES_SDIV(ahrs_impl.imu_rate, ahrs_impl.imu_rate, gyro_lowpass_filter);
  //#else
  } else {
	  RATES_COPY(ahrs_impl.imu_rate, omega);
  //#endif
  }

  /* add correction     */
  RATES_ADD(omega, ahrs_impl.rate_correction);
  /* and zeros it */
  INT_RATES_ZERO(ahrs_impl.rate_correction);

  /* integrate quaternion */
  INT32_QUAT_INTEGRATE_FI(ahrs_impl.ltp_to_imu_quat, ahrs_impl.high_rez_quat, omega, AHRS_PROPAGATE_FREQUENCY);
  INT32_QUAT_NORMALIZE(ahrs_impl.ltp_to_imu_quat);

  set_body_state_from_quat();

}



struct Int32Quat residualqfromacc;

void ahrs_update_accel(void) {

  // c2 = ltp z-axis in imu-frame
  struct Int32RMat ltp_to_imu_rmat;
  INT32_RMAT_OF_QUAT(ltp_to_imu_rmat, ahrs_impl.ltp_to_imu_quat);
  struct Int32Vect3 c2 = { RMAT_ELMT(ltp_to_imu_rmat, 0,2),
                           RMAT_ELMT(ltp_to_imu_rmat, 1,2),
                           RMAT_ELMT(ltp_to_imu_rmat, 2,2)};
  struct Int32Vect3 residual;

  struct Int32Vect3 pseudo_gravity_measurement;

  if (ahrs_impl.correct_gravity && ahrs_impl.ltp_vel_norm_valid) {
    /*
     * centrifugal acceleration in body frame
     * a_c_body = omega x (omega x r)
     * (omega x r) = tangential velocity in body frame
     * a_c_body = omega x vel_tangential_body
     * assumption: tangential velocity only along body x-axis
     */

    // FIXME: check overflows !
#define COMPUTATION_FRAC 16

    const struct Int32Vect3 vel_tangential_body = {ahrs_impl.ltp_vel_norm >> COMPUTATION_FRAC, 0, 0};
    struct Int32Vect3 acc_c_body;
    VECT3_RATES_CROSS_VECT3(acc_c_body, body_rates_i, vel_tangential_body);
    INT32_VECT3_RSHIFT(acc_c_body, acc_c_body, INT32_SPEED_FRAC+INT32_RATE_FRAC-INT32_ACCEL_FRAC-COMPUTATION_FRAC);

    /* convert centrifucal acceleration from body to imu frame */
    struct Int32Vect3 acc_c_imu;
    INT32_RMAT_VMULT(acc_c_imu, imu.body_to_imu_rmat, acc_c_body);

    /* and subtract it from imu measurement to get a corrected measurement of the gravitiy vector */
    INT32_VECT3_DIFF(pseudo_gravity_measurement, imu.accel, acc_c_imu);
  } else {
    VECT3_COPY(pseudo_gravity_measurement, imu.accel);
  }

  /* compute the residual of the pseudo gravity vector in imu frame */
  INT32_VECT3_CROSS_PRODUCT(residual, pseudo_gravity_measurement, c2);


  int32_t inv_weight;
  if (ahrs_impl.use_gravity_heuristic) {
    /* heuristic on acceleration norm */

    /* FIR filtered pseudo_gravity_measurement */
    static struct Int32Vect3 filtered_gravity_measurement = {0, 0, 0};
    VECT3_SMUL(filtered_gravity_measurement, filtered_gravity_measurement, 7);
    VECT3_ADD(filtered_gravity_measurement, pseudo_gravity_measurement);
    VECT3_SDIV(filtered_gravity_measurement, filtered_gravity_measurement, 8);

    int32_t acc_norm;
    INT32_VECT3_NORM(acc_norm, filtered_gravity_measurement);
    const int32_t acc_norm_d = ABS(ACCEL_BFP_OF_REAL(9.81)-acc_norm);
    inv_weight = Chop(50*acc_norm_d/ACCEL_BFP_OF_REAL(9.81), 1, 50);
  }
  else {
    inv_weight = 1;
  }

  // residual FRAC : ACCEL_FRAC + TRIG_FRAC = 10 + 14 = 24
  // rate_correction FRAC = RATE_FRAC = 12
  // 2^12 / 2^24 * 5e-2 = 1/81920
  ahrs_impl.rate_correction.p += -residual.x/82000/inv_weight;
  ahrs_impl.rate_correction.q += -residual.y/82000/inv_weight;
  ahrs_impl.rate_correction.r += -residual.z/82000/inv_weight;

  // residual FRAC = ACCEL_FRAC + TRIG_FRAC = 10 + 14 = 24
  // high_rez_bias = RATE_FRAC+28 = 40
  // 2^40 / 2^24 * 5e-6 = 1/3.05

  //  ahrs_impl.high_rez_bias.p += residual.x*3;
  //  ahrs_impl.high_rez_bias.q += residual.y*3;
  //  ahrs_impl.high_rez_bias.r += residual.z*3;

  ahrs_impl.high_rez_bias.p += residual.x/(2*inv_weight);
  ahrs_impl.high_rez_bias.q += residual.y/(2*inv_weight);
  ahrs_impl.high_rez_bias.r += residual.z/(2*inv_weight);

  residualqfromacc.qx = residual.x;
  residualqfromacc.qy = residual.y;
  residualqfromacc.qz = residual.z;

  /*                        */
  INT_RATES_RSHIFT(ahrs_impl.gyro_bias, ahrs_impl.high_rez_bias, 28);

}

struct Int32Quat residualq;
void ahrs_update_vicon(void) {

  // to align IMU with vicon:
  //INT32_QUAT_INV_COMP_NORM_SHORTEST(residualq, ahrs_impl.ltp_to_imu_quat, vicon_quat_i); //_b2c, _a2b, _a2c

  struct Int32Quat vicon_to_imu_quat_i;
  /* compute IMU orientation in Vicon frame */
  INT32_QUAT_COMP(vicon_to_imu_quat_i, vicon_quat_i, imu.body_to_imu_quat);
  /* compute the residual error */
  INT32_QUAT_INV_COMP_NORM_SHORTEST(residualq, ahrs_impl.ltp_to_imu_quat, vicon_to_imu_quat_i);


/*  struct Int32RMat ltp_to_imu_rmat;
  INT32_RMAT_OF_QUAT(ltp_to_imu_rmat, ahrs_impl.ltp_to_imu_quat);
  struct Int32Vect3 expected_imu;
  INT32_RMAT_VMULT(expected_imu, ltp_to_imu_rmat, ahrs_impl.mag_h);
*/
  struct Int32Vect3 residual;
  //INT32_VECT3_CROSS_PRODUCT(residual, imu.mag, expected_imu);

  residual.x = residualq.qx * 1024; // >> (INT32_QUAT_FRAC - INT32_MAG_FRAC);
  residual.y = residualq.qy * 1024; // >> (INT32_QUAT_FRAC - INT32_MAG_FRAC);
  residual.z = residualq.qz * 1024; // >> (INT32_QUAT_FRAC - INT32_MAG_FRAC);

  ahrs_impl.rate_correction.p += residual.x/32/16;
  ahrs_impl.rate_correction.q += residual.y/32/16;
  ahrs_impl.rate_correction.r += residual.z/32/16;


  ahrs_impl.high_rez_bias.p -= residual.x/32*1024;
  ahrs_impl.high_rez_bias.q -= residual.y/32*1024;
  ahrs_impl.high_rez_bias.r -= residual.z/32*1024;


  INT_RATES_RSHIFT(ahrs_impl.gyro_bias, ahrs_impl.high_rez_bias, 28);

}


void ahrs_update_mag(void) {
//#if AHRS_MAG_UPDATE_ALL_AXES
//  ahrs_update_mag_full();
//#else
  ahrs_update_mag_2d();
//#endif
}


static inline void ahrs_update_mag_full(void) {

  struct Int32RMat ltp_to_imu_rmat;
  INT32_RMAT_OF_QUAT(ltp_to_imu_rmat, ahrs_impl.ltp_to_imu_quat);

  struct Int32Vect3 expected_imu;
  INT32_RMAT_VMULT(expected_imu, ltp_to_imu_rmat, ahrs_impl.mag_h);

  struct Int32Vect3 residual;
  INT32_VECT3_CROSS_PRODUCT(residual, imu.mag, expected_imu);

  ahrs_impl.rate_correction.p += residual.x/32/16;
  ahrs_impl.rate_correction.q += residual.y/32/16;
  ahrs_impl.rate_correction.r += residual.z/32/16;


  ahrs_impl.high_rez_bias.p -= residual.x/32*1024;
  ahrs_impl.high_rez_bias.q -= residual.y/32*1024;
  ahrs_impl.high_rez_bias.r -= residual.z/32*1024;


  INT_RATES_RSHIFT(ahrs_impl.gyro_bias, ahrs_impl.high_rez_bias, 28);

}


static inline void ahrs_update_mag_2d(void) {

  struct Int32RMat ltp_to_imu_rmat;
  INT32_RMAT_OF_QUAT(ltp_to_imu_rmat, ahrs_impl.ltp_to_imu_quat);

  struct Int32Vect3 measured_ltp;
  INT32_RMAT_TRANSP_VMULT(measured_ltp, ltp_to_imu_rmat, imu.mag);

  struct Int32Vect3 residual_ltp =
    { 0,
      0,
      (measured_ltp.x * ahrs_impl.mag_h.y - measured_ltp.y * ahrs_impl.mag_h.x)/(1<<5)};

  struct Int32Vect3 residual_imu;
  INT32_RMAT_VMULT(residual_imu, ltp_to_imu_rmat, residual_ltp);

  // residual_ltp FRAC = 2 * MAG_FRAC = 22
  // rate_correction FRAC = RATE_FRAC = 12
  // 2^12 / 2^22 * 2.5 = 1/410

  //  ahrs_impl.rate_correction.p += residual_imu.x*(1<<5)/410;
  //  ahrs_impl.rate_correction.q += residual_imu.y*(1<<5)/410;
  //  ahrs_impl.rate_correction.r += residual_imu.z*(1<<5)/410;

  ahrs_impl.rate_correction.p += residual_imu.x/16;
  ahrs_impl.rate_correction.q += residual_imu.y/16;
  ahrs_impl.rate_correction.r += residual_imu.z/16;


  // residual_ltp FRAC = 2 * MAG_FRAC = 22
  // high_rez_bias = RATE_FRAC+28 = 40
  // 2^40 / 2^22 * 2.5e-3 = 655

  //  ahrs_impl.high_rez_bias.p -= residual_imu.x*(1<<5)*655;
  //  ahrs_impl.high_rez_bias.q -= residual_imu.y*(1<<5)*655;
  //  ahrs_impl.high_rez_bias.r -= residual_imu.z*(1<<5)*655;

  ahrs_impl.high_rez_bias.p -= residual_imu.x*1024;
  ahrs_impl.high_rez_bias.q -= residual_imu.y*1024;
  ahrs_impl.high_rez_bias.r -= residual_imu.z*1024;


  INT_RATES_RSHIFT(ahrs_impl.gyro_bias, ahrs_impl.high_rez_bias, 28);

}

void ahrs_update_gps(void) {
#if AHRS_GRAVITY_UPDATE_COORDINATED_TURN && USE_GPS
  if (gps.fix == GPS_FIX_3D) {
    ahrs_impl.ltp_vel_norm = SPEED_BFP_OF_REAL(gps.speed_3d / 100.);
    ahrs_impl.ltp_vel_norm_valid = TRUE;
  } else {
    ahrs_impl.ltp_vel_norm_valid = FALSE;
  }
#endif

#if AHRS_USE_GPS_HEADING && USE_GPS
  //got a 3d fix,ground speed > 0.5 m/s and course accuracy is better than 10deg
  if(gps.fix == GPS_FIX_3D &&
     gps.gspeed >= 500 &&
     gps.cacc <= RadOfDeg(10*1e7)) {

    // gps.course is in rad * 1e7, we need it in rad * 2^INT32_ANGLE_FRAC
    int32_t course = gps.course * ((1<<INT32_ANGLE_FRAC) / 1e7);

    /* the assumption here is that there is no side-slip, so heading=course */

    if (ahrs_impl.heading_aligned) {
      ahrs_update_heading(course);
    }
    else {
      /* hard reset the heading if this is the first measurement */
      ahrs_realign_heading(course);
    }
  }
#endif
}


void ahrs_update_heading(int32_t heading) {

  INT32_ANGLE_NORMALIZE(heading);

  // row 0 of ltp_to_body_rmat = body x-axis in ltp frame
  // we only consider x and y


  INT32_RMAT_OF_QUAT(ned_to_body_orientation_rmat_i, ned_to_body_orientation_quat_i);
  struct Int32RMat* ltp_to_body_rmat = &ned_to_body_orientation_rmat_i;
  struct Int32Vect2 expected_ltp =
    { RMAT_ELMT((*ltp_to_body_rmat), 0, 0),
      RMAT_ELMT((*ltp_to_body_rmat), 0, 1) };

  int32_t heading_x, heading_y;
  PPRZ_ITRIG_COS(heading_x, heading); // measured course in x-direction
  PPRZ_ITRIG_SIN(heading_y, heading); // measured course in y-direction

  // expected_heading cross measured_heading ??
  struct Int32Vect3 residual_ltp =
    { 0,
      0,
      (expected_ltp.x * heading_y - expected_ltp.y * heading_x)/(1<<INT32_ANGLE_FRAC)};

  struct Int32Vect3 residual_imu;
  struct Int32RMat ltp_to_imu_rmat;
  INT32_RMAT_OF_QUAT(ltp_to_imu_rmat, ahrs_impl.ltp_to_imu_quat);
  INT32_RMAT_VMULT(residual_imu, ltp_to_imu_rmat, residual_ltp);

  // residual FRAC = TRIG_FRAC + TRIG_FRAC = 14 + 14 = 28
  // rate_correction FRAC = RATE_FRAC = 12
  // 2^12 / 2^28 * 4.0 = 1/2^14
  // (1<<INT32_ANGLE_FRAC)/2^14 = 1/4
  ahrs_impl.rate_correction.p += residual_imu.x/4;
  ahrs_impl.rate_correction.q += residual_imu.y/4;
  ahrs_impl.rate_correction.r += residual_imu.z/4;


  /* crude attempt to only update bias if deviation is small
   * e.g. needed when you only have gps providing heading
   * and the inital heading is totally different from
   * the gps course information you get once you have a gps fix.
   * Otherwise the bias will be falsely "corrected".
   */
  int32_t sin_max_angle_deviation;
  PPRZ_ITRIG_SIN(sin_max_angle_deviation, TRIG_BFP_OF_REAL(RadOfDeg(5.)));
  if (ABS(residual_ltp.z) < sin_max_angle_deviation)
  {
    // residual_ltp FRAC = 2 * TRIG_FRAC = 28
    // high_rez_bias = RATE_FRAC+28 = 40
    // 2^40 / 2^28 * 2.5e-4 = 1
    ahrs_impl.high_rez_bias.p -= residual_imu.x*(1<<INT32_ANGLE_FRAC);
    ahrs_impl.high_rez_bias.q -= residual_imu.y*(1<<INT32_ANGLE_FRAC);
    ahrs_impl.high_rez_bias.r -= residual_imu.z*(1<<INT32_ANGLE_FRAC);

    INT_RATES_RSHIFT(ahrs_impl.gyro_bias, ahrs_impl.high_rez_bias, 28);
  }
}

void ahrs_realign_heading(int32_t heading) {

  struct Int32Quat ltp_to_body_quat = ned_to_body_orientation_quat_i;

  /* quaternion representing only the heading rotation from ltp to body */
  struct Int32Quat q_h_new;
  q_h_new.qx = 0;
  q_h_new.qy = 0;
  PPRZ_ITRIG_SIN(q_h_new.qz, heading/2);
  PPRZ_ITRIG_COS(q_h_new.qi, heading/2);

  /* quaternion representing current heading only */
  struct Int32Quat q_h;
  QUAT_COPY(q_h, ltp_to_body_quat);
  q_h.qx = 0;
  q_h.qy = 0;
  INT32_QUAT_NORMALIZE(q_h);

  /* quaternion representing rotation from current to new heading */
  struct Int32Quat q_c;
  INT32_QUAT_INV_COMP_NORM_SHORTEST(q_c, q_h, q_h_new);

  /* correct current heading in body frame */
  struct Int32Quat q;
  INT32_QUAT_COMP_NORM_SHORTEST(q, q_c, ltp_to_body_quat);
  QUAT_COPY(ltp_to_body_quat, q);

  /* compute ltp to imu rotations */
  INT32_QUAT_COMP(ahrs_impl.ltp_to_imu_quat, ltp_to_body_quat, imu.body_to_imu_quat);

  /* Set state */
  QUAT_COPY(ned_to_body_orientation_quat_i, ltp_to_body_quat);

  ahrs_impl.heading_aligned = TRUE;
}


/* Rotate angles and rates from imu to body frame and set state */
__attribute__ ((always_inline)) static inline void set_body_state_from_quat(void) {
  /* Compute LTP to BODY quaternion */
  struct Int32Quat ltp_to_body_quat;
  INT32_QUAT_COMP_INV(ltp_to_body_quat, ahrs_impl.ltp_to_imu_quat, imu.body_to_imu_quat);
  /* Set state */
  QUAT_COPY(ned_to_body_orientation_quat_i, ltp_to_body_quat);

  /* compute body rates */
  struct Int32Rates body_rate;
  INT32_RMAT_TRANSP_RATEMULT(body_rate, imu.body_to_imu_rmat, ahrs_impl.imu_rate);
  /* Set state */
  RATES_COPY(body_rates_i, body_rate);
}


