#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "rosnao_bridge/image_subscriber.hpp"

rosnao::ImageSubscriber<rosnao::kQVGA> *sub_qvga = nullptr;
rosnao::ImageSubscriber<rosnao::kVGA> *sub_vga = nullptr;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosnao_bridge_test");
    ros::NodeHandle nh;

    if (argc != 3)
    {
        std::cerr << "shm_id, res {1=QVGA, 2=VGA}" << std::endl;
        return 1;
    }

    std::cout << argv[1] <<std::endl;
    std::cout << argv[2] <<std::endl;
    const std::string shm_id = argv[1];
    const int res = std::stoi(argv[2]);

    if (res == rosnao::kVGA)
    {
        sub_vga = new rosnao::ImageSubscriber<rosnao::kVGA>(shm_id);
        while (ros::ok())
        {
            auto p = sub_vga->get();
            if (p.second)
            {
                std::cout << p.first.rows << p.first.cols << std::endl;

                cv::imshow("test", p.first);
                cv::waitKey(3);
            }
            // ros::spinOnce();
        }
        delete sub_vga;
    }
    else if (res == rosnao::kQVGA)
    {
        sub_qvga = new rosnao::ImageSubscriber<rosnao::kQVGA>(shm_id);
        while (ros::ok())
        {
            auto p = sub_qvga->get();
            if (p.second)
            {
                cv::imshow("test", p.first);
                cv::waitKey(3);
            }
            // ros::spinOnce();
        }
        delete sub_qvga;
    }
    else
    {
        assert(false); // res must be 1 (QVGA) or 2 (VGA)
    }

    ros::spin();
    return 1;
}