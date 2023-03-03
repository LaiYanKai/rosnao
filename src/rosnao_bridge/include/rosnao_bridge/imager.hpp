#include 



template <int res>
    class ImageSubscriber
    {
    private:
        using _img_t = transport::Image<res>;
        _img_t *shm_img;
        std::string shm_id;

    public:
        ImageSubscriber(const std::string &shm_id) : shm_id(shm_id)
        {
            for (int attempt = 0;; ++attempt)
            {
                try
                {
                    boost::interprocess::shared_memory_object shm(
                        boost::interprocess::open_only,
                        shm_id,
                        boost::interprocess::read_only);

                    // Map the whole shared memory in this process
                    boost::interprocess::mapped_region region(shm, boost::interprocess::read_only);

                    // Get the address of the mapped region
                    void *addr = region.get_address();

                    // Construct the shared structure in memory
                    shm_img = static_cast<_img_t *>(addr);
                }
                catch (boost::interprocess::interprocess_exception &ex)
                {
                    std::cout << "Attempt " << attempt << ": " << ex.what() << std::endl;
                    if (attempt >= 5)
                        delete this;
                    else
                        continue;
                }
            }
        }

        ~ImageSubscriber()
        {
            // boost::interprocess::shared_memory_object::remove(shm_id.c_str());
        }

    };