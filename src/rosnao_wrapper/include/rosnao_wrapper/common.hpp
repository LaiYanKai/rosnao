#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <signal.h>
#include <iostream>

#ifndef ROSNAO_WRAPPER_COMMON_HPP
#define ROSNAO_WRAPPER_COMMON_HPP

#define _def_interrupt                                         \
    volatile std::sig_atomic_t processing_interrupted = false; \
    extern "C" void interrupt_processing(int)                  \
    {                                                          \
        processing_interrupted = true;                         \
        std::cout << "Keyboard interrupt" << std::endl;        \
    }
#define _uninstall_interrupt std::signal(SIGINT, SIG_DFL);
#define _init_interrupt             \
    processing_interrupted = false; \
    std::signal(SIGINT, &interrupt_processing);
#define _no_interrupt processing_interrupted == false
#endif