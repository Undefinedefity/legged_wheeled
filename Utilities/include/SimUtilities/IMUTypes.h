#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include "cppTypes.h"

struct ImuData {
  Vec3<double> acc;
  Vec3<double> gyro;
  Quat<double> quat;
  Vec3<double> base_p;
  Vec3<double> base_v;
};

#endif // IMU_TYPES_H