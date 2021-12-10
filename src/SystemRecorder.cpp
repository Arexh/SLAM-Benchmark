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

    void SystemRecorder::recordSystemStart()
    {
        SystemInfoManager::startMonitor();
    }

    void SystemRecorder::recordSystemStop()
    {
        m_info_record = SystemInfoManager::stopMonitor();
    }

    void SystemRecorder::recordThreadCreate(const std::string thread_name)
    {
        m_thread_map[thread_name].start_time = Utility::getCurrentMillisecond();
    }

    void SystemRecorder::recordThreadDestory(const std::string thread_name)
    {
        m_thread_map[thread_name].end_time = Utility::getCurrentMillisecond();
    }

    void SystemRecorder::recordThreadProcessStart(const string thread_name)
    {
        m_thread_map[thread_name].temp_time = Utility::getCurrentMillisecond();
    }

    void SystemRecorder::recordThreadProcessStop(const string thread_name)
    {
        SystemThread system_thread = m_thread_map[thread_name];
        system_thread.time_meter.update(Utility::getCurrentMillisecond() - system_thread.temp_time);
    }

}