#include "rosnao_wrapper/motion.hpp"

volatile std::sig_atomic_t processing_interrupted = false;
extern "C" void interrupt_processing(int)
{
    processing_interrupted = true;
    std::cout << "Keyboard interrupt" << std::endl;
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cerr << "Wrong Arguments for ROSNAO_WRAPPER MOTION: Usage nao_ip, shm_id" << std::endl;
        // The same shm_id for the same cam cannot be reused. It will cause other publishers to stop working.
        // 
        return 1;
    }
    processing_interrupted = false;
    std::signal(SIGINT, &interrupt_processing);

    const std::string nao_ip = argv[1];
    const std::string shm_id = argv[2];
    // const int res = std::stoi(argv[3]);
    // const int cam = std::stoi(argv[4]);

    AL::ALMotionProxy motion(nao_ip);
    AL::ALRobotPostureProxy robotPosture(nao_ip);

    robotPosture.goToPosture("StandInit", 0.5f);
    motion.moveInit();

    AL::ALValue move_cfg;
    move_cfg.arraySetSize(0);
    AL::ALValue move_cfg_entry;
    move_cfg_entry.array("MaxStepFrequency", 0.5);
    move_cfg.arrayPush(move_cfg_entry);
    motion.moveTo(0.3, 0, 1.57, move_cfg);
    motion.moveTo(0.0, 0.3, -1.57);

    // while (!processing_interrupted)
    // {
    // }

    std::signal(SIGINT, SIG_DFL);
    return 0;
}