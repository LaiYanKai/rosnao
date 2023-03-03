#include "rosnao_common/image.hpp"
#include <opencv2/opencv.hpp>

#ifndef ROSNAO_BRIDGE_IMAGE_SUBSCRIBER_HPP
#define ROSNAO_BRIDGE_IMAGE_SUBSCRIBER_HPP

namespace rosnao
{
    template <int res>
    class ImageSubscriber
    {
    private:
        using _img_t = transport::Image<res>;
        _img_t *shm_img;
        boost::interprocess::mapped_region region;
        cv::Mat mat;
        std::string shm_id;
        uint32_t seq = 0;

    public:
        ImageSubscriber(const std::string &shm_id) : shm_id(shm_id), mat(_img_t::height, _img_t::width, CV_8UC1)
        {
            for (int attempt = 0;; ++attempt)
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
                    shm_img = static_cast<_img_t *>(addr);
                    std::cout << "Subscribe attempt " << attempt << " succeeded." << std::endl;

                    break;
                }
                catch (boost::interprocess::interprocess_exception &ex)
                {
                    std::cout << "Failed subscribing attempt " << attempt << ": " << ex.what() << std::endl;
                    ros::Duration(1).sleep();
                }
            }
        }

        ~ImageSubscriber()
        { // boost::interprocess::shared_memory_object::remove(shm_id.c_str());
        }

        // returns true if there is a new picture.
        std::pair<cv::Mat, bool> get()
        {
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(shm_img->mutex);
            if (shm_img->seq == seq)
                return std::make_pair(mat, false);
            seq = shm_img->seq;

            const size_t size = _img_t::height * _img_t::width * _img_t::channels;
            auto &shm_data = shm_img->data;
            for (size_t i = 0; i < size; ++i)
                mat.data[i] = shm_data[i];

            return std::make_pair(mat, true);
        }
    };
}
#endif