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
        m_start_time = Utility::getCurrentMillisecond();
        SystemInfoManager::startMonitor();
    }

    void SystemRecorder::recordSystemStop()
    {
        m_end_time = Utility::getCurrentMillisecond();
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

    nlohmann::ordered_json SystemRecorder::summary()
    {
        nlohmann::ordered_json summary = {
            {"system name", ToString(m_system_name)},
            {"start time", m_start_time},
            {"end time", m_end_time},
            {"interval", m_end_time - m_start_time},
            {"cpu", m_info_record->cpu_percent_meter.summaryStatistics("%")},
            {"virtual memory", m_info_record->virtual_memory_meter.summaryStatistics()},
            {"physical memory", m_info_record->physical_memory_meter.summaryStatistics()}
        };
        if (SystemInfoManager::isCPUPowerAvailable()) {
            summary["cpu power"] = m_info_record->cpu_power_meter.summaryStatistics();
        } else {
            summary["cpu power"] = "unavailable";
        }
        if (SystemInfoManager::isGPUPowerAvailable()) {
            summary["gpu power"] = m_info_record->gpu_power_meter.summaryStatistics();
        } else {
            summary["gpu power"] = "unavailable";
        }
        if (SystemInfoManager::isSOCPowerAvailable()) {
            summary["soc power"] = m_info_record->soc_power_meter.summaryStatistics();
        } else {
            summary["soc power"] = "unavailable";
        }
        return summary;
    }
}