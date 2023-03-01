#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#ifndef ROSNAO_TRANSPORT_IMAGER_HPP
#define ROSNAO_TRANSPORT_IMAGER_HPP
namespace rosnao::transport
{
    // const int kYuvColorSpace = 0;
    // const int kRGBColorSpace = 11;
    // const int kVGA = 2;
    // const int kQVGA = 1;

    struct AbstractImage
    {
        boost::interprocess::interprocess_mutex mutex;
        uint32_t seq = 0;
        int32_t sec = 0, usec = 0;
    };

    struct ImageQVGA : AbstractImage
    {
        inline const static int width = 320;
        inline const static int height = 240; 
        inline const static int channels = 1;
        uint8_t data[width*height*channels];
    };
    struct ImageVGA : AbstractImage
    {
        inline const static int width = 640;
        inline const static int height = 480; 
        inline const static int channels = 1;
        uint8_t data[width*height*channels];
    };

    class ImageSubscriber
    {

    };
}
#endif