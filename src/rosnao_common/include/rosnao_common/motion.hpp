#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "rosnao_common/common.hpp"

#ifndef ROSNAO_COMMON_MOTION_HPP
#define ROSNAO_COMMON_MOTION_HPP
namespace rosnao
{
    enum Joint
    {
        Null,
        HeadYaw,
    };

    std::string to_string(const Joint &joint)
    {
        if (joint == Joint::HeadYaw)
            return "HeadYaw";
        else
            return "";
    }

    namespace transport
    {
        enum MotionFunction
        {
            Null,
            MoveTo,
            SetAngle,
        };
        struct SHMMotion
        {
            boost::interprocess::interprocess_mutex mutex;
            const float x = 0, y = 0, angle = 0, speed = 0;
            uint32_t seq = 0;
            Joint joint = Joint::Null;
            MotionFunction func = MotionFunction::Null;
            bool block = true;
        };
    }
}
#endif