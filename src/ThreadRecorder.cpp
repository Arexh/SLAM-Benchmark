#include "ThreadRecorder.h"
#include <chrono>
#include <sys/stat.h>
#include <unistd.h>

namespace SLAM_Benchmark
{
    inline uint64_t getCurrentMillisecond();

    /*
        struct timespec {
            time_t   tv_sec;        / seconds
            long     tv_nsec;       // nanoseconds
        };
    */
    struct timespec getThreadTime();

    void ThreadRecorder::recordThreadCreate()
    {
        start_time = getCurrentMillisecond();
    }

    void ThreadRecorder::recordThreadDestory()
    {
        end_time = getCurrentMillisecond();
        thread_time = getThreadTime().tv_nsec / 1000.0;
    }

    void ThreadRecorder::recordThreadProcessStart()
    {
        temp_time = getCurrentMillisecond();
    }

    void ThreadRecorder::recordThreadProcessStop()
    {
        time_meter.update(getCurrentMillisecond() - temp_time);
    }

    inline uint64_t getCurrentMillisecond()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }

    // copy from: https://stackoverflow.com/questions/44916362/how-can-i-measure-cpu-time-of-a-specific-set-of-threads
    struct timespec getThreadTime()
    {
        struct timespec currTime;
        clockid_t threadClockId;
        //! Get thread clock Id
        pthread_getcpuclockid(pthread_self(), &threadClockId);
        //! Using thread clock Id get the clock time
        clock_gettime(threadClockId, &currTime);
        return currTime;
    }
}