#include "Utility.h"
#include "stdio.h"

namespace SLAM_Benchmark
{
    std::atomic_bool Utility::thread_flag(true);

    // copy from: https://stackoverflow.com/questions/35525543/multithread-program-in-c-shows-the-same-performance-as-a-serial-one
    double Utility::simpleComputationTask(unsigned long long loops)
    {
        volatile double x;
        for (unsigned long long i = 0; i < loops; i++)
        {
            x = sin(sqrt(i) / i * 3.14159);
        }
        return x;
    }

    void Utility::timedTask(const uint32_t interval,
                            const std::function<void()> callback)
    {
        while (thread_flag)
        {
            boost::asio::io_service io;
            boost::asio::deadline_timer t(io, boost::posix_time::milliseconds(interval));
            t.async_wait([&](const boost::system::error_code &)
                         { callback(); });
            io.run();
        }
    }
}