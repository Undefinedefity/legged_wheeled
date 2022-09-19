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
};

struct ActuatorCmd
{
  uint8_t mode[4];
  
  double tau_ref[6] = {0};  // leftleg: hip, knee, wheel; rightleg: hip, knee, wheel

  double q_des_wheel[2];
  double q_des_hip[2];
  double q_des_knee[2];

  double qd_des_wheel[2];
  double qd_des_hip[2];
  double qd_des_knee[2];

  double kp_wheel[2];
  double kp_hip[2];
  double kp_knee[2];

  double kd_wheel[2];
  double kd_hip[2];
  double kd_knee[2];
};

#endif // MOTOR_TYPES_H