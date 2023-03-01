#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include "rosnao/transport/imager.hpp"

#ifndef ROSNAO_IMAGER_HPP
#define ROSNAO_IMAGER_HPP
namespace rosnao
{
    class ImagerPublisher
    {
        private:
            int sub_id;
        public:
    };

}
#endif