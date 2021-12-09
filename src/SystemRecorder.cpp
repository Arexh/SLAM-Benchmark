#include "SystemRecorder.h"
#include "Utility.h"

namespace SLAM_Benchmark
{

    void SystemRecorder::registerThread(const std::string thread_name)
    {
        SystemThread system_thread;
        system_thread.thread_name = thread_name;
        m_thread_map[thread_name] = system_thread;
    }

    void SystemRecorder::startThreadRecord(const std::string thread_name)
    {
        m_thread_map[thread_name].start_time = Utility::getCurrentMillisecond();
    }

    void SystemRecorder::stopThreadRecord(const std::string thread_name)
    {
        SystemThread thread = m_thread_map[thread_name];
        time_t interval = Utility::getCurrentMillisecond() - thread.start_time;
        thread.time_meter.update(interval);
    }

}