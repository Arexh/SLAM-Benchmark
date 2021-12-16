#include "ThreadRecorder.h"
#include <chrono>
#include <sys/stat.h>
#include <unistd.h>

namespace SLAM_Benchmark
{
    inline uint64_t getCurrentNanosecond();

    /*
        struct timespec {
            time_t   tv_sec;        / seconds
            long     tv_nsec;       // nanoseconds
        };
    */
    struct timespec getThreadTime();

    void ThreadRecorder::recordThreadCreate()
    {
        start_time = getCurrentNanosecond();
    }

    void ThreadRecorder::recordThreadDestory()
    {
        end_time = getCurrentNanosecond();
        thread_time = getThreadTime().tv_nsec;
    }

    void ThreadRecorder::recordThreadProcessStart()
    {
        temp_time = getCurrentNanosecond();
    }

    void ThreadRecorder::recordThreadProcessStop()
    {
        time_meter.update(getCurrentNanosecond() - temp_time);
    }

    inline uint64_t getCurrentNanosecond()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
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