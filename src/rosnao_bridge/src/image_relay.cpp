#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "rosnao_bridge/image_subscriber.hpp"

rosnao::ImageSubscriber<rosnao::kQVGA> *sub_qvga = nullptr;
rosnao::ImageSubscriber<rosnao::kVGA> *sub_vga = nullptr;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosnao_image_relay");
    ros::NodeHandle nh;

    if (argc != 4)
    {
        std::cerr << "shm_id, res {1=QVGA, 2=VGA}, topic" << std::endl;
        return 1;
    }

    std::cout << ros::this_node::getName()
              << ": shm_id[" << argv[1]
              << "] res[" << argv[2]
              << "] topic[" << argv[3]
              << "]" << std::endl;

    const std::string shm_id = argv[1];
    const int res = std::stoi(argv[2]);
    const std::string topic = argv[3];

    if (res == rosnao::kVGA)
        sub_vga = new rosnao::ImageSubscriber<rosnao::kVGA>(shm_id);
    else if (res == rosnao::kQVGA)
        sub_qvga = new rosnao::ImageSubscriber<rosnao::kQVGA>(shm_id);
    else
        assert(false); // res must be 1 (QVGA) or 2 (VGA)

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(topic, 1);

    while (ros::ok())
    {
        /* 
        std::pair<cv::Mat, bool> p;
        if (res == rosnao::kVGA)
            p = sub_vga->getCvMat();
        else if (res == rosnao::kQVGA)
            p = sub_qvga->getCvMat(); 
        
        if (p.second == false)
            continue; // don't publish anything if nothing is received

        cv::imshow("test", p.first);
        cv::waitKey(3);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", p.first).toImageMsg();
        pub.publish(msg);
        */

        std::pair<sensor_msgs::ImageConstPtr, bool> p;
        if (res == rosnao::kVGA)
            p = sub_vga->getImageMsg();
        else if (res == rosnao::kQVGA)
            p = sub_qvga->getImageMsg();     

        if (p.second == false)
            continue; // don't publish anything if nothing is received
        
        pub.publish(p.first);

        ros::spinOnce();
    }

    if (res == rosnao::kVGA)
        delete sub_vga;
    else if (res == rosnao::kQVGA)
        delete sub_qvga;

    ros::spin();
    return 0;
}