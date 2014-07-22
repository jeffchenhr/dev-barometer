#ifndef SUPERVISION_H
#define SUPERVISION_H

#include "std.h"
#include "airframe.h"
#include "math/pprz_algebra_int.h"


#define MAX_PPRZ 9600
#define MIN_PPRZ -MAX_PPRZ

struct Int32HybridCommand {
  int32_t roll;
  int32_t pitch;
  int32_t yaw;
  int32_t thrust;
  struct Int32Rates rates;
};

struct Int32MotorCommand {
  int32_t u[6];
  //int32_t elevons[2];
  struct Int32Rates rates;
};

extern int32_t asctec_esc_version_number;

void actuators_set(bool_t motors_on, int32_t commands[]);
void actuators_set_i2c_new(bool_t motors_on, int32_t commands_u[]);

extern struct Int32Quat stabilization_att_sum_err_quat;
extern struct Int32Eulers stabilization_att_sum_err;

extern int32_t stabilization_att_fb_cmd[COMMANDS_NB];
extern int32_t stabilization_att_ff_cmd[COMMANDS_NB];
extern int32_t stabilization_cmd[COMMANDS_NB];

#define IERROR_SCALE 1024
#define GAIN_PRESCALER_FF 48
#define GAIN_PRESCALER_P 48
#define GAIN_PRESCALER_D 48
#define GAIN_PRESCALER_I 48


struct Int32AttitudeGains {
  struct Int32Vect3  p;
  struct Int32Vect3  d;
  struct Int32Vect3  dd;
  struct Int32Vect3  i;
};

extern struct Int32AttitudeGains stabilization_gains;

struct Int32AttitudeGainMatrixRow {
	  struct Int32Vect3  p;
	  struct Int32Vect3  d;
	  struct Int32Vect3  i;
	  int32_t t; // thrust command gain
	  int32_t offset; // for setting neutral value of elevons
	};

extern struct Int32AttitudeGainMatrixRow gain_matrix[6];
extern struct Int32AttitudeGainMatrixRow gain_matrix_manual[6];

#endif /* SUPERVISION_H */
