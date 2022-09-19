#include "ControlData.h"
#include "ParamHandler.hpp"
#include <iostream>

bool RobotParameter::getParamsFromYAML(const char *filename)
{
  ParamHandler param_handler((std::string(filename)));
  bool _successful = true;

  _successful &= param_handler.getVector(std::string("jointlimit"), jointlimit);
  _successful &= param_handler.getVector(std::string("Kp"), Kp);
  _successful &= param_handler.getVector(std::string("Kd"), Kd);
  std::cout << "joint limit &  Kp, Kd load succeed" << std::endl;

  _successful &= param_handler.getValue(std::string("desHeight"), desHeight);
  _successful &= param_handler.getValue(std::string("velocityRef"), velocityRef);
  _successful &= param_handler.getValue(std::string("controlMode"), controlMode);
  std::cout << "desired hiehgt load succeed" << std::endl;

  if (!_successful)
  {
    throw std::runtime_error("init param failed ...");
  }

  return _successful;
}