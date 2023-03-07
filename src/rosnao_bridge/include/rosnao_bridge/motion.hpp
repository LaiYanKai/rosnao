#include "rosnao_common/motion.hpp"

#ifndef ROSNAO_BRIDGE_MOTION_RELAY_HPP
#define ROSNAO_BRIDGE_MOTION_RELAY_HPP
#define get_scoped_lock boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(shm_motion->mutex)

namespace rosnao
{
    // There should only be one instance of this class.
    // Provides some moving functions for basic VSLAM implementation
    class Motion
    {
    private:
        std::string shm_id;
        transport::SHMMotion *shm_motion;
        boost::interprocess::mapped_region region;
        uint32_t seq = 0;

        void blockUntilProxyCompletes()
        {
            while (ros::ok())
            {
                get_scoped_lock;
                if (seq != shm_motion->seq)
                {
                    seq = shm_motion->seq;
                    break;
                }
            }
        }

    public:
        Motion &operator=(const Motion &) = delete;
        Motion(const Motion &) = delete;

        Motion(const std::string &shm_id) : shm_id(shm_id)
        {
            for (int attempt = 0; ros::ok(); ++attempt)
            {
                try
                {
                    boost::interprocess::shared_memory_object shm(
                        boost::interprocess::open_only,
                        shm_id.c_str(),
                        boost::interprocess::read_write);

                    // Map the whole shared memory in this process
                    region = boost::interprocess::mapped_region(shm, boost::interprocess::read_write);

                    // Get the address of the mapped region
                    void *addr = region.get_address();

                    // Construct the shared structure in memory
                    shm_motion = static_cast<transport::SHMMotion *>(addr);
                    std::cout << "Motion: Proxy contact attempt " << attempt << " succeeded." << std::endl;

                    break;
                }
                catch (boost::interprocess::interprocess_exception &ex)
                {
                    std::cout << "Motion: Failed proxy contact attempt " << attempt << ": " << ex.what() << std::endl;
                    ros::Duration(1).sleep();
                }
            }
        }

        // Walk to x(m), y(m), and yaw(rad) pose in the robot's frame, treating the current pose as (0,0,0).
        // Blocks until the walk is complete.
        void moveTo(const float &x, const float &y, const float &yaw)
        {
            { // acquires a lock and sends the command to shm
                get_scoped_lock;
                seq = ++shm_motion->seq;
                shm_motion->func = transport::MotionFunction::MoveTo;
                shm_motion->x = x;
                shm_motion->y = y;
                shm_motion->angle = yaw;
            }

            // blocks execution until the proxy completes
            blockUntilProxyCompletes();
        }

        // Moves a joint to an absolute angle (rad), at fraction_max_speed (between 0 and 1)
        // By default, this function blocks until the joint is within 0.04 rad of the angle (the internal controller has no steady-state control). Set block to false to unblock.
        void setAngle(const rosnao::Joint &joint, const float &angle, const float &fraction_max_speed, const bool &block = true)
        {
            { // acquires a lock and sends the command to shm
                get_scoped_lock;
                seq = ++shm_motion->seq;
                shm_motion->func = transport::MotionFunction::SetAngle;
                shm_motion->joint = joint;
                shm_motion->angle = angle;
                shm_motion->speed = fraction_max_speed;
                shm_motion->block = block;
            }

            // blocks execution until the proxy completes
            blockUntilProxyCompletes();
        }

        // stiffens the joints and stands up.
        void wakeUp()
        {
            { // acquires a lock and sends the command to shm
                get_scoped_lock;
                seq = ++shm_motion->seq;
                shm_motion->func = transport::MotionFunction::WakeUp;
            }

            // blocks execution until the proxy completes
            blockUntilProxyCompletes();
        }

        // resets the posture of the robot to a walking position.
        void moveInit()
        {
            { // acquires a lock and sends the command to shm
                get_scoped_lock;
                seq = ++shm_motion->seq;
                shm_motion->func = transport::MotionFunction::MoveInit;
            }

            // blocks execution until the proxy completes
            blockUntilProxyCompletes();
        }

        // unstiffens the joints and crouch. Helps to prevent motors from overheating.
        void rest()
        {
            { // acquires a lock and sends the command to shm
                get_scoped_lock;
                seq = ++shm_motion->seq;
                shm_motion->func = transport::MotionFunction::Rest;
            }

            // blocks execution until the proxy completes
            blockUntilProxyCompletes();
        }
    };
}

#endif