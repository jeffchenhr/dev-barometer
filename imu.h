/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 * @file subsystems/imu.h
 * Inertial Measurement Unit interface.
 */

#ifndef IMU_H
#define IMU_H

#define PERIODIC_FREQUENCY 1000


#define IMU_GYRO_P_NEUTRAL 0
#define IMU_GYRO_Q_NEUTRAL 0
#define IMU_GYRO_R_NEUTRAL -7
#define IMU_GYRO_P_SENS 4.359
#define IMU_GYRO_P_SENS_NUM 4359
#define IMU_GYRO_P_SENS_DEN 1000
#define IMU_GYRO_Q_SENS 4.359
#define IMU_GYRO_Q_SENS_NUM 4359
#define IMU_GYRO_Q_SENS_DEN 1000
#define IMU_GYRO_R_SENS 4.359
#define IMU_GYRO_R_SENS_NUM 4359
#define IMU_GYRO_R_SENS_DEN 1000
#define IMU_ACCEL_X_NEUTRAL 0
#define IMU_ACCEL_Y_NEUTRAL 0
#define IMU_ACCEL_Z_NEUTRAL 0
#define IMU_ACCEL_X_SENS 4.92297204167
#define IMU_ACCEL_X_SENS_NUM 7222
#define IMU_ACCEL_X_SENS_DEN 1467
#define IMU_ACCEL_Y_SENS 4.88255070466
#define IMU_ACCEL_Y_SENS_NUM 45022
#define IMU_ACCEL_Y_SENS_DEN 9221
#define IMU_ACCEL_Z_SENS 4.86606505696
#define IMU_ACCEL_Z_SENS_NUM 20491
#define IMU_ACCEL_Z_SENS_DEN 4211
#define IMU_MAG_X_NEUTRAL 0
#define IMU_MAG_Y_NEUTRAL 0
#define IMU_MAG_Z_NEUTRAL 0
#define IMU_MAG_X_SENS 1
#define IMU_MAG_X_SENS_NUM 1
#define IMU_MAG_X_SENS_DEN 1
#define IMU_MAG_Y_SENS 1
#define IMU_MAG_Y_SENS_NUM 1
#define IMU_MAG_Y_SENS_DEN 1
#define IMU_MAG_Z_SENS 1
#define IMU_MAG_Z_SENS_NUM 1
#define IMU_MAG_Z_SENS_DEN 1

#define IMU_MAG_X_SIGN 1
#define IMU_MAG_Y_SIGN 1
#define IMU_MAG_Z_SIGN 1

#define IMU_GYRO_P_SIGN 1
#define IMU_GYRO_Q_SIGN 1
#define IMU_GYRO_R_SIGN 1

#define IMU_ACCEL_X_SIGN -1
#define IMU_ACCEL_Y_SIGN -1
#define IMU_ACCEL_Z_SIGN -1
/*
#define IMU_ACCEL_X_SIGN 1
#define IMU_ACCEL_Y_SIGN 1
#define IMU_ACCEL_Z_SIGN 1
*/

#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
//#include "generated/airframe.h"

/* must be defined by underlying hardware */
extern void imu_impl_init(void);
extern void imu_periodic(void);

/** abstract IMU interface providing fixed point interface  */
struct Imu {
  struct Int32Rates gyro;             ///< gyroscope measurements
  struct Int32Vect3 accel;            ///< accelerometer measurements
  struct Int32Vect3 mag;              ///< magnetometer measurements
  struct Int32Rates gyro_prev;        ///< previous gyroscope measurements
  struct Int32Vect3 accel_prev;       ///< previous accelerometer measurements
  struct Int32Rates gyro_neutral;     ///< gyroscope bias
  struct Int32Vect3 accel_neutral;    ///< accelerometer bias
  struct Int32Vect3 mag_neutral;      ///< magnetometer neutral readings (bias)
  struct Int32Rates gyro_unscaled;    ///< unscaled gyroscope measurements
  struct Int32Vect3 accel_unscaled;   ///< unscaled accelerometer measurements
  struct Int32Vect3 mag_unscaled;     ///< unscaled magnetometer measurements
  struct Int32Quat  body_to_imu_quat; ///< rotation from body to imu frame as a unit quaternion
  struct Int32RMat  body_to_imu_rmat; ///< rotation from body to imu frame as a rotation matrix
};

/** abstract IMU interface providing floating point interface  */
struct ImuFloat {
  struct FloatRates   gyro;
  struct FloatVect3   accel;
  struct FloatVect3   mag;
  struct FloatRates   gyro_prev;
  struct FloatEulers  body_to_imu_eulers;
  struct FloatQuat    body_to_imu_quat;
  struct FloatRMat    body_to_imu_rmat;
  uint32_t sample_count;
};

extern void imu_float_init(struct ImuFloat* imuf);

/** global IMU state */
extern struct Imu imu;

/* underlying hardware */
#ifdef IMU_TYPE_H
#include IMU_TYPE_H
#endif

extern void imu_init(void);

// rotations around x,y,z (in imu frame)
// QUADSHOT:
//#define IMU_BODY_TO_IMU_PHI RadOfDeg(  0. )
//#define IMU_BODY_TO_IMU_THETA RadOfDeg( 90. )
//#define IMU_BODY_TO_IMU_PSI RadOfDeg(  180. )
// SERTAC:
#define IMU_BODY_TO_IMU_PHI RadOfDeg(  180. )
#define IMU_BODY_TO_IMU_THETA RadOfDeg( 0. )
#define IMU_BODY_TO_IMU_PSI RadOfDeg(  -90. )

/*
#define IMU_BODY_TO_IMU_PHI RadOfDeg(   0. )
#define IMU_BODY_TO_IMU_THETA RadOfDeg(   90. )
#define IMU_BODY_TO_IMU_PSI RadOfDeg(  0. )
*/

#if !defined IMU_BODY_TO_IMU_PHI && !defined IMU_BODY_TO_IMU_THETA && !defined IMU_BODY_TO_IMU_PSI
#define IMU_BODY_TO_IMU_PHI   0
#define IMU_BODY_TO_IMU_THETA 0
#define IMU_BODY_TO_IMU_PSI   0
#endif

#if !defined IMU_GYRO_P_NEUTRAL && !defined IMU_GYRO_Q_NEUTRAL && !defined IMU_GYRO_R_NEUTRAL
#define IMU_GYRO_P_NEUTRAL 0
#define IMU_GYRO_Q_NEUTRAL 0
#define IMU_GYRO_R_NEUTRAL 0
#endif

#if !defined IMU_ACCEL_X_NEUTRAL && !defined IMU_ACCEL_Y_NEUTRAL && !defined IMU_ACCEL_Z_NEUTRAL
#define IMU_ACCEL_X_NEUTRAL 0
#define IMU_ACCEL_Y_NEUTRAL 0
#define IMU_ACCEL_Z_NEUTRAL 0
#endif


#ifndef ImuScaleGyro
#define ImuScaleGyro(_imu) {					\
    RATES_COPY(_imu.gyro_prev, _imu.gyro);				\
    _imu.gyro.p = ((_imu.gyro_unscaled.p - _imu.gyro_neutral.p)*IMU_GYRO_P_SIGN*IMU_GYRO_P_SENS_NUM)/IMU_GYRO_P_SENS_DEN; \
    _imu.gyro.q = ((_imu.gyro_unscaled.q - _imu.gyro_neutral.q)*IMU_GYRO_Q_SIGN*IMU_GYRO_Q_SENS_NUM)/IMU_GYRO_Q_SENS_DEN; \
    _imu.gyro.r = ((_imu.gyro_unscaled.r - _imu.gyro_neutral.r)*IMU_GYRO_R_SIGN*IMU_GYRO_R_SENS_NUM)/IMU_GYRO_R_SENS_DEN; \
  }
#endif


#ifndef ImuScaleAccel
#define ImuScaleAccel(_imu) {					\
    VECT3_COPY(_imu.accel_prev, _imu.accel);				\
    _imu.accel.x = ((_imu.accel_unscaled.x - _imu.accel_neutral.x)*IMU_ACCEL_X_SIGN*IMU_ACCEL_X_SENS_NUM)/IMU_ACCEL_X_SENS_DEN; \
    _imu.accel.y = ((_imu.accel_unscaled.y - _imu.accel_neutral.y)*IMU_ACCEL_Y_SIGN*IMU_ACCEL_Y_SENS_NUM)/IMU_ACCEL_Y_SENS_DEN; \
    _imu.accel.z = ((_imu.accel_unscaled.z - _imu.accel_neutral.z)*IMU_ACCEL_Z_SIGN*IMU_ACCEL_Z_SENS_NUM)/IMU_ACCEL_Z_SENS_DEN; \
  }
#endif

#ifndef ImuScaleMag
#if defined IMU_MAG_45_HACK
#define ImuScaleMag(_imu) {						\
    int32_t msx = ((_imu.mag_unscaled.x - _imu.mag_neutral.x) * IMU_MAG_X_SIGN * IMU_MAG_X_SENS_NUM) / IMU_MAG_X_SENS_DEN; \
    int32_t msy = ((_imu.mag_unscaled.y - _imu.mag_neutral.y) * IMU_MAG_Y_SIGN * IMU_MAG_Y_SENS_NUM) / IMU_MAG_Y_SENS_DEN; \
    _imu.mag.x = msx - msy;						\
    _imu.mag.y = msx + msy;						\
    _imu.mag.z = ((_imu.mag_unscaled.z - _imu.mag_neutral.z) * IMU_MAG_Z_SIGN * IMU_MAG_Z_SENS_NUM) / IMU_MAG_Z_SENS_DEN; \
  }
#elif defined IMU_MAG_X_CURRENT_COEF && defined IMU_MAG_Y_CURRENT_COEF && defined IMU_MAG_Z_CURRENT_COEF
#define ImuScaleMag(_imu) {						\
    struct Int32Vect3 _mag_correction;                                  \
    _mag_correction.x = (int32_t) (IMU_MAG_X_CURRENT_COEF * (float) electrical.current); \
    _mag_correction.y = (int32_t) (IMU_MAG_Y_CURRENT_COEF * (float) electrical.current); \
    _mag_correction.z = (int32_t) (IMU_MAG_Z_CURRENT_COEF * (float) electrical.current); \
    _imu.mag.x = (((_imu.mag_unscaled.x - _mag_correction.x) - _imu.mag_neutral.x) * IMU_MAG_X_SIGN * IMU_MAG_X_SENS_NUM) / IMU_MAG_X_SENS_DEN; \
    _imu.mag.y = (((_imu.mag_unscaled.y - _mag_correction.y) - _imu.mag_neutral.y) * IMU_MAG_Y_SIGN * IMU_MAG_Y_SENS_NUM) / IMU_MAG_Y_SENS_DEN; \
    _imu.mag.z = (((_imu.mag_unscaled.z - _mag_correction.z) - _imu.mag_neutral.z) * IMU_MAG_Z_SIGN * IMU_MAG_Z_SENS_NUM) / IMU_MAG_Z_SENS_DEN; \
 }
#else
#define ImuScaleMag(_imu) {                                             \
    _imu.mag.x = ((_imu.mag_unscaled.x - _imu.mag_neutral.x) * IMU_MAG_X_SIGN * IMU_MAG_X_SENS_NUM) / IMU_MAG_X_SENS_DEN; \
    _imu.mag.y = ((_imu.mag_unscaled.y - _imu.mag_neutral.y) * IMU_MAG_Y_SIGN * IMU_MAG_Y_SENS_NUM) / IMU_MAG_Y_SENS_DEN; \
    _imu.mag.z = ((_imu.mag_unscaled.z - _imu.mag_neutral.z) * IMU_MAG_Z_SIGN * IMU_MAG_Z_SENS_NUM) / IMU_MAG_Z_SENS_DEN; \
  }
#endif //IMU_MAG_45_HACK
#endif //ImuScaleMag


#endif /* IMU_H */
