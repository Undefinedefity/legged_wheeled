#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <stdint.h>
#include "cTypes.h"

struct ActuatorData
{
  double q_hip[2];
  double q_knee[2];
  double q_wheel[2];

  double qd_hip[2];
  double qd_knee[2];
  double qd_wheel[2];

  double qdd_hip[2];
  double qdd_knee[2];
  double qdd_wheel[2];

  double tau_hip[2];
  double tau_knee[2];
  double tau_wheel[2];

  // robot arm state
  double q_arm[4];
  double qd_arm[4];
  double qdd_arm[4];
  double tau_arm[4];
};

struct ActuatorCmd
{
  double q_des_hip[2];
  double q_des_knee[2];
  double q_des_wheel[2];

  double qd_des_hip[2];
  double qd_des_knee[2];
  double qd_des_wheel[2];

  double kp_hip[2];
  double kp_knee[2];
  double kp_wheel[2];

  double kd_hip[2];
  double kd_knee[2];
  double kd_wheel[2];

  double tau_hip_ff[2]; // tau_hip_ff[i]: 0-fl; 1-fr; 2-rl; 3-rr
  double tau_knee_ff[2];
  double tau_wheel_ff[2];
  
  double tau_ref[6];

  // robot arm cmd
  double q_des_arm[4];
  double qd_des_arm[4];
  double kp_arm[4];
  double kd_arm[4];
  double tau_arm_ff[4];
};

#endif // MOTOR_TYPES_H