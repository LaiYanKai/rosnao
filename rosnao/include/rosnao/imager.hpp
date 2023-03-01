#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include "rosnao/transport/imager.hpp"

#ifndef ROSNAO_IMAGER_HPP
#define ROSNAO_IMAGER_HPP
namespace rosnao
{
    // http://doc.aldebaran.com/2-8/naoqi/vision/alvideodevice-api.html#alvideodevice-api
    // http://doc.aldebaran.com/2-8/ref/libalvision/alvisiondefinitions_8h.html
    // http://doc.aldebaran.com/2-8/ref/libalvision/classAL_1_1ALImage.html
    // http://doc.aldebaran.com/2-8/ref/libalvision/alimage__opencv_8h_source.html
    // http://docs.ros.org/en/noetic/api/sensor_msgs/html/image__encodings_8h.html

    template <int res>
    class ImagePublisher
    {
    private:
        using _img_t = transport::Image<res>;
        AL::ALVideoDeviceProxy proxy;
        _img_t *shm_img;
        void _fill(const int32_t &sec, const int32_t &usec, const uint8_t *const &img_data)
        {
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(shm_img->mutex); // this will release when _fill goes out of scope
            ++shm_img->seq;
            shm_img->sec = sec;
            shm_img->usec = usec;

            const size_t size = _img_t::height * _img_t::width * _img_t::channels;
            auto &shm_data = shm_img->data;
            for (size_t i = 0; i < size; ++i)
                shm_data[i] = img_data[i];
            std::cout << "Published Image " << shm_img->seq << std::endl;
        }
        std::string sub_id, shm_id;
        int fps, cam;

    public:
        //     ip: ip address
        // shm_id: shared memory label, also used as handle(string) for proxy
        //    fps: frames per second
        //    cam: kTopCamera (0) or kBottomCamera (1)
        ImagePublisher(const std::string &ip, const std::string &shm_id, const int &fps, const int &cam)
            : proxy(ip), sub_id(proxy.subscribe(shm_id, AL::kQVGA, AL::kYuvColorSpace, fps)), shm_id(shm_id), fps(fps), cam(cam)
        {
            try
            {
                boost::interprocess::shared_memory_object shm(
                    boost::interprocess::create_only,
                    shm_id.c_str(),
                    boost::interprocess::read_write);

                shm.truncate(sizeof(_img_t));
                boost::interprocess::mapped_region region(shm, boost::interprocess::read_write);
                void *addr = region.get_address();
                shm_img = new (addr) _img_t;
                
                proxy.setActiveCamera(sub_id, cam);
            }
            catch (boost::interprocess::interprocess_exception &ex)
            {
                std::cout << ex.what() << std::endl;
                delete this;
            }
        }
        ~ImagePublisher()
        {
            boost::interprocess::shared_memory_object::remove(shm_id.c_str());
            proxy.unsubscribe(sub_id);
        }
        void pub()
        {
            // Image Data in ALValue
            // [0]: width.
            // [1]: height.
            // [2]: number of layers.
            // [3]: ColorSpace.
            // [4]: time stamp from qi::Clock (seconds).
            // [5]: time stamp from qi::Clock (microseconds).
            // [6]: binary array of size height * width * nblayers containing image data.
            // [7]: camera ID (kTop=0, kBottom=1).
            // [8]: left angle (radian).
            // [9]: topAngle (radian).
            // [10]: rightAngle (radian).
            // [11]: bottomAngle (radian).
            std::cout << "Request Image" << std::endl;
            AL::ALValue results = proxy.getImageRemote(sub_id);

            if (results.getSize() == 0)
            {
                std::cout << "Cannot retrieve Image(results empty)" << std::endl;
                return;
            }
            const uint8_t *img_data = static_cast<const uint8_t *>(results[6].GetBinary());
            if (img_data == NULL)
            {
                std::cout << "Cannot retrieve Image(null)" << std::endl;
                return;
            }

            _fill(results[4], results[5], img_data);

            proxy.releaseImage(sub_id);
        }
    };

}
#endif