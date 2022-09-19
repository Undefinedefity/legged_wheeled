//
// Created by wenchun on 3/26/21.
//

#ifndef XIAOTIANARM_USERINTERFACE_H
#define XIAOTIANARM_USERINTERFACE_H

#include "joystick.h"
#include "ControlData.h"
#include <memory>

using namespace std;

class UserInterface {
public:
  UserInterface(RobotParameter *param);

  void update();

private:
  RobotParameter *param;
  GamepadCommand gameCmd;
  JoystickEvent event;
  shared_ptr<Joystick> joystick;
  int iter = 0;
};


#endif
