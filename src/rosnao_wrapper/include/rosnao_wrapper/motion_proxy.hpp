
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include "rosnao_common/motion.hpp"
#include "rosnao_wrapper/common.hpp"

#ifndef ROSNAO_WRAPPER_MOTION_HPP
#define ROSNAO_WRAPPER_MOTION_HPP

// http://doc.aldebaran.com/2-8/naoqi/motion/control-walk-api.html
// http://doc.aldebaran.com/2-8/naoqi/motion/control-joint-api.html
// http://doc.aldebaran.com/2-8/naoqi/motion/control-walk.html
// http://doc.aldebaran.com/2-8/ref/libalvalue/alvalue_8h_source.html

namespace rosnao
{
    class MotionProxy
    {
    private:
        AL::ALMotionProxy proxy;
        AL::ALRobotPostureProxy proxy_posture;
        AL::ALValue move_cfg;

    public:
        MotionProxy(const std::string &nao_ip)
            : proxy(nao_ip), proxy_posture(nao_ip)
        {
            // Makes robot stand up, stiffen its joints, and prepare to walk
            proxy.wakeUp();
            proxy.moveInit();

            // Set move config so it moves slower
            move_cfg = proxy.getMoveConfig("Default");
            //[["MaxStepX", 0.04], ["MaxStepY", 0.14], ["MaxStepTheta", 0.349066], ["MaxStepFrequency", 1], ["StepHeight", 0.02], ["TorsoWx", 0], ["TorsoWy", 0]]
            // std::cout << move_cfg << std::endl;
            assert(move_cfg[3][0] == std::string("MaxStepFrequency"));
            move_cfg[3][1] = 0.1; // set maxstepfrequency  to move slower
        }
        ~MotionProxy() { proxy.rest(); }

        // stiffens the joints and stands up.
        void wakeUp() { proxy.wakeUp(); }

        // resets the posture of the robot to a walking position.
        void moveInit() { proxy.moveInit(); }

        // unstiffens the joints and crouch. Helps to prevent motors from overheating.
        void rest() { proxy.rest(); }

        // Moves robot at x (m/s), y (m/s) and yaw (m/s) velocities in robot's frame.
        void move(const float &x_vel, const float &y_vel, const float &yaw_vel) { proxy.move(x_vel, y_vel, yaw_vel); }

        // Moves robot x (m), y (m), yaw (radians) in robot's frame from current robot pose.
        // This function is blocking.
        // http://doc.aldebaran.com/2-8/naoqi/motion/control-walk-api.html
        void moveTo(const float &x, const float &y, const float &yaw) { proxy.moveTo(x, y, yaw, move_cfg); }

        // Moves robot at x, y and yaw normalised velocities in robot's frame.
        void moveToward(const float &x_nvel, const float &y_nvel, const float &yaw_nvel) { proxy.moveToward(x_nvel, y_nvel, yaw_nvel); }

        // Moves a joint to an absolute angle (rad), at fraction_max_speed (between -1 and 1)
        // By default, this function blocks until the joint is within 0.04 rad of the angle (the internal controller has no steady-state control). Set block to false to unblock.
        // http://doc.aldebaran.com/2-8/naoqi/motion/control-joint-api.html
        // does no argument validation (done in sdk).
        void setAngle(const rosnao::Joint &joint, const float &angle, const float &fraction_max_speed, const bool &block = true)
        {
            const AL::ALValue _joint = rosnao::to_string(joint);
            proxy.setAngles(_joint, angle, fraction_max_speed);
            if (block)
                while (std::abs(proxy.getAngles(_joint, true)[0] - angle) > 0.04)
                    ;
        }
    };

}
#endif