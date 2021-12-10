#include "ThreadRecorder.h"
#include "Utility.h"

namespace SLAM_Benchmark
{
    void ThreadRecorder::recordThreadCreate()
    {
        start_time = Utility::getCurrentMillisecond();
    }

    void ThreadRecorder::recordThreadDestory()
    {
        end_time = Utility::getCurrentMillisecond();
        thread_time = Utility::getThreadTime().tv_nsec / 1000.0;
    }

    void ThreadRecorder::recordThreadProcessStart()
    {
        temp_time = Utility::getCurrentMillisecond();
    }

    void ThreadRecorder::recordThreadProcessStop()
    {
        time_meter.update(Utility::getCurrentMillisecond() - temp_time);
    }
}