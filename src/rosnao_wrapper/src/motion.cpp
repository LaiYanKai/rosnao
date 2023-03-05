#include "rosnao_wrapper/motion.hpp"

volatile std::sig_atomic_t processing_interrupted = false;
extern "C" void interrupt_processing(int)
{
    processing_interrupted = true;
    std::cout << "Keyboard interrupt" << std::endl;
}

int main(int argc, char **argv)
{
    // if (argc != 5)
    // {
    //     std::cout << "nao_ip, shm_id, res {1=QVGA, 2=VGA}, cam {0=top,1=bottom}" << std::endl;
    //     // The same shm_id for the same cam cannot be reused. It will cause other publishers to stop working.
    //     // 
    //     return 1;
    // }
    processing_interrupted = false;
    std::signal(SIGINT, &interrupt_processing);

    // const std::string nao_ip = argv[1];
    // const std::string shm_id = argv[2];
    // const int res = std::stoi(argv[3]);
    // const int cam = std::stoi(argv[4]);

    while (!processing_interrupted)
    {
        
    }

    std::signal(SIGINT, SIG_DFL);
    return 0;
}