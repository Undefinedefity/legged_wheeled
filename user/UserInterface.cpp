//
// Created by wenchun on 3/26/21.
//

#include "UserInterface.h"

UserInterface::UserInterface(RobotParameter *param) : param(param)
{

    joystick = std::make_shared<Joystick>("/dev/input/js0");
    if (!joystick->isFound())
        printf("joystick not found!\n");
    gameCmd.zero();
}

void UserInterface::update()
{
    // update joystick state at 50Hz
    if (iter % 10 == 0)
        joystick->updateCommand(&event, gameCmd);

    // control mode
    if (gameCmd.START)
    {
        param->controlMode = 0; // wip_lqr control
    }
    else if (gameCmd.BACK)
    {
        param->controlMode = 2; // wbc control
    }
    // else if (gameCmd.X)
    // {
    //     param->ctrl_mode = 2; // PID control
    // }
    // else if (gameCmd.Y)
    // {
    //     param->ctrl_mode = 3; // MPC control
    // }

    // if (gameCmd.A)
    // {
    //     param->EnableWheelTorque = true;
    // }
    // if (gameCmd.B)
    // {
    //     param->ChickenHeadOn = true; // Set to chicken head mode
    // }
    
    // command
    double filter = 0.05;
    param->velocityRef = filter * gameCmd.leftStickAnalog(1) + (1 - filter) * param->velocityRef;
    // param->des_translation[1] += 0.0001 * gameCmd.rightStickAnalog(1);

    // param->des_translation[2] += 0.0001 * (gameCmd.LB ? 1 : 0);
    // param->des_translation[2] -= 0.0001 * (gameCmd.RB ? 1 : 0);

    // param->des_rotation[0] += 0.0005 * ((gameCmd.Dpad(0) > 0) ? 1 : 0);
    // param->des_rotation[0] -= 0.0005 * ((gameCmd.Dpad(0) < 0) ? 1 : 0);    
    // param->des_rotation[1] += 0.0005 * ((gameCmd.Dpad(1) > 0) ? 1 : 0);
    // param->des_rotation[1] -= 0.0005 * ((gameCmd.Dpad(1) < 0) ? 1 : 0);

    // param->des_rotation[2] += 0.0005 * (gameCmd.LT ? 1 : 0);
    // param->des_rotation[2] -= 0.0005 * (gameCmd.RT ? 1 : 0);

    std::cout << "LeftStickAnalog [1]: " << gameCmd.leftStickAnalog(1) << std::endl
              << "RightStickAnalog [1]: " << gameCmd.rightStickAnalog(1) << std::endl
              << "Dpad[0]: " << gameCmd.Dpad(0) << std::endl
              << "Dpad[1]: " << gameCmd.Dpad(1) << std::endl
              << "RT: " << gameCmd.RT << std::endl
              << "LT: " << gameCmd.LT << std::endl;

    iter++;
}