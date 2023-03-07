#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "rosnao_bridge/motion.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosnao_test_motion");
    ros::NodeHandle nh;

    if (argc != 2)
    {
        std::cerr << "Wrong arguments for IMAGE_RELAY. Usage: shm_id" << std::endl;
        return 1;
    }

    std::cout << ros::this_node::getName()
              << ": shm_id[" << argv[1]
              << "]" << std::endl;

    const std::string shm_id = argv[1];

    rosnao::Motion motion(shm_id); // stiffens the robot's joints and makes the robot assume a walking posture


    // motion.wakeUp(); // called in ctor of rosnao::Motion.
    // motion.moveInit(); // called in ctor of rosnao::Motion. call again if need to reset walking posture.

    // motion.moveTo(0.3, 0.0, 1.57);                      // go forward 30cm and turn 90deg left (blocking call)
    // motion.setAngle(rosnao::HeadYaw, 1.57, 0.05, true); // rotate head to 90deg left  (blocking call)
    // motion.setAngle(rosnao::HeadYaw, -1.57, 0.1, true); // rotate head to 90deg right, faster (blocking call)
    // motion.moveTo(0.0, 0.3, -1.57);                     // go left 30cm, and turn 90deg right (go to origin) (blocking call)
    motion.setAngle(rosnao::HeadYaw, 1.57, 0.05, false); // rotate head to 90deg left (non blocking)
    motion.moveTo(0.3, 0.0, 1.57);                       // go forward 30cm and turn 90deg left (head turns while moving, block until robot walked to position)
    motion.setAngle(rosnao::HeadYaw, -1.57, 0.1, false); // rotate head to 90deg right, faster (non blocking, doesn't care if the head reached 90deg left)
    motion.moveTo(0.0, 0.3, -1.57);                      // go left 30cm, and turn 90deg right (go to origin) (head turns while moving, block until robot walked to position)
    
    motion.rest(); // tells the robot to rest to prevent robot overheating

    return 0;
}