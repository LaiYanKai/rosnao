#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "rosnao_common/types.hpp"

#ifndef ROSNAO_COMMON_IMAGE_HPP
#define ROSNAO_COMMON_IMAGE_HPP
namespace rosnao::transport
{

    struct AbstractImage
    {
        boost::interprocess::interprocess_mutex mutex;
        uint32_t seq = 0;
        int32_t sec = 0, usec = 0;
    };

    template <int res>
    struct Image : AbstractImage
    {
    };

    template <>
    struct Image<kVGA> : AbstractImage
    {
        inline const static int width = 320;
        inline const static int height = 240;
        inline const static int channels = 1;
        uint8_t data[width * height * channels];
    };

    template <>
    struct Image<kQVGA> : AbstractImage
    {
        inline const static int width = 640;
        inline const static int height = 480;
        inline const static int channels = 1;
        uint8_t data[width * height * channels];
    };

    
}
#endif