#include "rosnao_wrapper/motion_proxy.hpp"

_def_interrupt;

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cerr << "Wrong Arguments for ROSNAO_WRAPPER MOTION: Usage nao_ip, shm_id" << std::endl;
        // The same shm_id for the same cam cannot be reused. It will cause other publishers to stop working.
        //
        return 1;
    }
    _init_interrupt;

    const std::string nao_ip = argv[1];
    const std::string shm_id = argv[2];

    // ========== Create Shared Memory =================
    boost::interprocess::mapped_region region; // cannot be deleted (will cause segmentation fault)
    using _shm_struct = rosnao::transport::SHMMotion;
    _shm_struct *shm_motion;

    try
    {
        boost::interprocess::shared_memory_object::remove(shm_id.c_str());

        boost::interprocess::shared_memory_object shm(
            boost::interprocess::open_or_create,
            shm_id.c_str(),
            boost::interprocess::read_write);

        shm.truncate(sizeof(_shm_struct));
        region = boost::interprocess::mapped_region(shm, boost::interprocess::read_write);
        void *addr = region.get_address();
        shm_motion = new (addr)(_shm_struct);
    }
    catch (boost::interprocess::interprocess_exception &ex)
    {
        std::cerr << "Boost Interprocess Exception: " << ex.what() << std::endl;
        boost::interprocess::shared_memory_object::remove(shm_id.c_str());
        return 1;
    }

    // ========== Create Proxy and Listen to Shared Memory =================
    try
    {
        rosnao::MotionProxy motion_proxy(nao_ip);
        uint32_t seq = 0;
        while (_no_interrupt)
        {
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(shm_motion->mutex);
            if (seq == shm_motion->seq)
                continue;
            seq = ++shm_motion->seq;

            if (shm_motion->func == rosnao::transport::MotionFunction::Move)
                motion_proxy.move(shm_motion->x, shm_motion->y, shm_motion->angle);
            else if (shm_motion->func == rosnao::transport::MotionFunction::MoveInit)
                motion_proxy.moveInit();
            else if (shm_motion->func == rosnao::transport::MotionFunction::MoveTo)
                motion_proxy.moveTo(shm_motion->x, shm_motion->y, shm_motion->angle);
            else if (shm_motion->func == rosnao::transport::MotionFunction::MoveToward)
                motion_proxy.moveToward(shm_motion->x, shm_motion->y, shm_motion->angle);
            else if (shm_motion->func == rosnao::transport::MotionFunction::Rest)
                motion_proxy.rest();
            else if (shm_motion->func == rosnao::transport::MotionFunction::SetAngle)
                motion_proxy.setAngle(shm_motion->joint, shm_motion->angle, shm_motion->speed, shm_motion->block);
            else if (shm_motion->func == rosnao::transport::MotionFunction::WakeUp)
                motion_proxy.wakeUp();
        }
    }
    catch (...)
    {
    }

    _uninstall_interrupt;
    boost::interprocess::shared_memory_object::remove(shm_id.c_str());
    return 0;
}