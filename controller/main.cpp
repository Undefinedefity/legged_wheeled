#include <csignal>
#include "RobotController.h"


bool stop = false;

void handler(int)
{
	std::cout << "will exit..." << std::endl;
	stop = true;
}

int main(){
    signal(SIGINT, handler);

    RobotController robotCtrl;
    std::cout << "Successfully initiate robot controller! " << std::endl;

    while (!stop)
    {
        if (robotCtrl._sharedMemory().waitForRobotWithTimeout(2, 0)){
            robotCtrl.run();
            robotCtrl._sharedMemory().controllerIsDone();
        }
    }

    robotCtrl.~RobotController();
    std::cout << "!!! Controller Destruct Finished !!!" << std::endl;
    return 0;
}