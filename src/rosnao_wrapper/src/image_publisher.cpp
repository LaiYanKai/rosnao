#include "rosnao_wrapper/image_publisher.hpp"

rosnao::ImagePublisher<rosnao::kVGA> *pub_vga = nullptr;
rosnao::ImagePublisher<rosnao::kQVGA> *pub_qvga = nullptr;

volatile std::sig_atomic_t processing_interrupted = false;
extern "C" void interrupt_processing(int)
{
    processing_interrupted = true;
    std::cout << "Keyboard interrupt" << std::endl;
}

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        std::cerr << "Wrong arguments for IMAGE_PUB. Usage: nao_ip, shm_id, res {1=QVGA, 2=VGA}, cam {0=top,1=bottom}" << std::endl;
        // The same shm_id for the same cam cannot be reused. It will cause other publishers to stop working.
        // 
        return 1;
    }
    processing_interrupted = false;
    std::signal(SIGINT, &interrupt_processing);

    const std::string nao_ip = argv[1];
    const std::string shm_id = argv[2];
    const int res = std::stoi(argv[3]);
    const int cam = std::stoi(argv[4]);

    std::cout << "IMAGE_PUBLISHER: shm_id[" << shm_id
              << "] NAO_IP[" << nao_ip
              << "] RES[" << res
              << "] CAM[" << cam
              << "]" << std::endl;

    assert(cam == 0 || cam == 1); // cam must be 0 (top) or 1 (bottom).
    if (res == rosnao::kVGA)
    {
        pub_vga = new rosnao::ImagePublisher<rosnao::kVGA>(nao_ip, shm_id, 30, cam);
        while (!processing_interrupted)
        {
            pub_vga->pub();
        }
        delete pub_vga;
    }
    else if (res == rosnao::kQVGA)
    {
        pub_qvga = new rosnao::ImagePublisher<rosnao::kQVGA>(nao_ip, shm_id, 30, cam);
        while (!processing_interrupted)
        {
            pub_qvga->pub();
        }
        delete pub_qvga;
    }
    else
    {
        assert(false); // res must be 1 (QVGA) or 2 (VGA)
    }

    std::signal(SIGINT, SIG_DFL);
    return 0;
}