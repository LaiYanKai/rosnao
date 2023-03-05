#include "rosnao_common/image.hpp"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#ifndef ROSNAO_BRIDGE_IMAGE_SUBSCRIBER_HPP
#define ROSNAO_BRIDGE_IMAGE_SUBSCRIBER_HPP

namespace rosnao
{
    template <int res>
    class ImageSubscriber
    {
    private:
        using _img_t = transport::SHMImage<res>;
        _img_t *shm_img;
        boost::interprocess::mapped_region region;
        cv::Mat mat;
        sensor_msgs::ImagePtr msg;
        std::string shm_id;
        uint32_t seq = 0;

    public:
        ImageSubscriber(const std::string &shm_id, const std::string &frame_id)
            : shm_id(shm_id), mat(_img_t::height, _img_t::width, CV_8UC1)
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

            // init msg
            msg = boost::make_shared<sensor_msgs::Image>();
            msg->header.frame_id = frame_id;
            msg->height = _img_t::height;
            msg->width = _img_t::width;
            msg->encoding = "mono8";
            msg->is_bigendian = false;
            msg->step = _img_t::width * _img_t::channels;
            size_t size = msg->step * msg->height;
            msg->data.resize(size);
        }

        ~ImageSubscriber()
        { // boost::interprocess::shared_memory_object::remove(shm_id.c_str());
        }

        // returns the image as a cv::Mat.
        // returns true if there is a new picture from the stream, false otherwise
        std::pair<cv::Mat, bool> getCvMat()
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

        // returns the image as a ImageConstPtr.
        // returns true if there is a new picture from the stream, false otherwise
        std::pair<sensor_msgs::ImageConstPtr, bool> getImageMsg()
        {
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(shm_img->mutex);
            if (shm_img->seq == seq)
                return std::make_pair(msg, false);

            auto &header = msg->header;
            header.seq = seq = shm_img->seq;
            header.stamp.sec = shm_img->sec;
            header.stamp.nsec = shm_img->usec * 1000;

            const size_t size = _img_t::height * _img_t::width * _img_t::channels;
            auto &shm_data = shm_img->data;
            // for (size_t i = 0; i < size; ++i)
            //     msg->data[i] = shm_data[i];
            memcpy((uint8_t *)(&msg->data[0]), shm_data, size);

            // cv_bridge::CvImage(std_msgs::Header(), "mono8", p.first).toImageMsg()

            return std::make_pair(msg, true);
        }
    };
}
#endif