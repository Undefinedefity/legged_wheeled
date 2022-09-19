
#ifndef SHARED_MEMORY_MESSAGE_H
#define SHARED_MEMORY_MESSAGE_H

#include <iostream>
#include <mutex>
#include "SharedMemory.h"
#include <vector>

#include "RobotCommData.h"
// #include "EstimatedState.h"

struct EstimatePos
{
    float pos[3] = {0., 0., 0.};
};

struct BaseTruth
{
    Eigen::Vector3d basePos_truth;
    Eigen::Vector3d baseVel_truth;
};

/*!
 * A SimulatorSyncronizedMessage is stored in shared memory and is accessed by
 * both the simulator and the robot The simulator and robot take turns have
 * exclusive access to the entire message. The intended sequence is:
 *  - robot: waitForRobot()
 *  - simulator: *simulates robot* (simulator can read/write, robot cannot do
 * anything)
 *  - simulator: simDone()
 *  - simulator: waitForRobot()
 *  - robot: *runs controller*    (robot can read/write, simulator cannot do
 * anything)
 *  - robot: robotDone();
 *  - robot: waitForRobot()
 *  ...
 */

class SharedMemoryInterface
{
public:
    void init()
    {
        ctrlToRobotSemaphore.init(0);
        robotToCtrlSemaphore.init(0);
    }

    void waitForRobot() { robotToCtrlSemaphore.decrement(); }

    void robotIsDone() { robotToCtrlSemaphore.increment(); }

    void waitForController() { ctrlToRobotSemaphore.decrement(); }

    void controllerIsDone() { ctrlToRobotSemaphore.increment(); }

    bool waitForRobotWithTimeout(unsigned int seconds, unsigned int nanoseconds)
    {
        bool ret_vel = robotToCtrlSemaphore.decrementTimeout(seconds, nanoseconds);
        while(robotToCtrlSemaphore.decrementTimeout(0, 100));
        return ret_vel;
    }

    bool waitForControllerWithTimeout(unsigned int seconds, unsigned int nanoseconds)
    {
        return ctrlToRobotSemaphore.decrementTimeout(seconds, nanoseconds);
    }

    MeasuredState measuredState;
    JointsCmd jointsCmd;
    // EstimatedState state_estimated;
    // EstimatePos estimatePos;
    BaseTruth baseTruth;
    
    bool ctrl_attached = false;

private:
    SharedMemorySemaphore ctrlToRobotSemaphore, robotToCtrlSemaphore;
};

#endif // SHARED_MEMORY_MESSAGE_H