#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "rosnao/types.hpp"

#ifndef ROSNAO_TRANSPORT_IMAGER_HPP
#define ROSNAO_TRANSPORT_IMAGER_HPP
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

    template <int res>
    class ImageSubscriber
    {
    private:
        using _img_t = transport::Image<res>;
        _img_t *shm_img;
        std::string shm_id;

    public:
        ImageSubscriber(const std::string &shm_id) : shm_id(shm_id)
        {
            for (int attempt = 0;; ++attempt)
            {
                try
                {
                    boost::interprocess::shared_memory_object shm(
                        boost::interprocess::open_only,
                        shm_id,
                        boost::interprocess::read_only);

                    // Map the whole shared memory in this process
                    boost::interprocess::mapped_region region(shm, boost::interprocess::read_only);

                    // Get the address of the mapped region
                    void *addr = region.get_address();

                    // Construct the shared structure in memory
                    shm_img = static_cast<_img_t *>(addr);
                }
                catch (boost::interprocess::interprocess_exception &ex)
                {
                    std::cout << "Attempt " << attempt << ": " << ex.what() << std::endl;
                    if (attempt >= 5)
                        delete this;
                    else
                        continue;
                }
            }
        }

        ~ImageSubscriber()
        {
            // boost::interprocess::shared_memory_object::remove(shm_id.c_str());
        }

        inline get()
    };
}
#endif