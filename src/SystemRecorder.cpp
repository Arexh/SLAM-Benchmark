#include "SystemRecorder.h"
#include "Utility.h"

namespace SLAM_Benchmark
{
    SystemRecorder::~SystemRecorder()
    {
        delete m_info_record;

        for (auto it = m_thread_map.begin(); it != m_thread_map.end(); it++)
        {
            delete it->second;
        }
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

    void SystemRecorder::addThreadRecord(ThreadRecorder* thread_recorder)
    {
        m_thread_map[thread_recorder->thread_name] = thread_recorder;
    }

    void SystemRecorder::addPublishRecord(ThreadRecorder* publish_recorder)
    {
        m_publish_record = publish_recorder;
    }

    nlohmann::ordered_json SystemRecorder::summary()
    {
        nlohmann::ordered_json summary = {
            {"SystemName", ToString(m_system_name)},
            {"CPU", m_info_record->cpu_percent_meter.summaryStatistics("%")},
            {"VirtualMemory", m_info_record->virtual_memory_meter.summaryStatistics()},
            {"PhysicalMemory", m_info_record->physical_memory_meter.summaryStatistics()},
            {"StartTime", m_start_time},
            {"EndTime", m_end_time},
            {"Interval", m_end_time - m_start_time},
            {"PublishTime", m_publish_record->time_meter.summaryStatistics()},
            {"AvgFPS", Utility::calculateFPS(m_publish_record->time_meter.getMean())}
        };

        if (SystemInfoManager::isCPUPowerAvailable()) {
            summary["CPUPower"] = m_info_record->cpu_power_meter.summaryStatistics();
        } else {
            summary["CPUPower"] = "Unavailable";
        }
        if (SystemInfoManager::isGPUPowerAvailable()) {
            summary["GPUPower"] = m_info_record->gpu_power_meter.summaryStatistics();
        } else {
            summary["GPUPower"] = "Unavailable";
        }
        if (SystemInfoManager::isSOCPowerAvailable()) {
            summary["SOCPower"] = m_info_record->soc_power_meter.summaryStatistics();
        } else {
            summary["SOCPower"] = "Unavailable";
        }

        nlohmann::ordered_json thread_info;
        for (auto it = m_thread_map.begin(); it != m_thread_map.end(); it++)
        {
            ThreadRecorder* record = it->second;
            thread_info[it->first] = {
                {"StartTime", record->start_time},
                {"EndTime", record->end_time},
                {"Interval", record->end_time - record->start_time},
                {"ThreadTime", record->thread_time},
                {"ProcessTime", record->time_meter.summaryStatistics()}
            };
        }
        summary["Threads"] = thread_info;

        nlohmann::ordered_json values;
        values["PublishTime"] = m_publish_record->time_meter.getValueList();

        values["CPU"] = m_info_record->cpu_percent_meter.getValueList();
        values["GPU"] = m_info_record->gpu_power_meter.getValueList();
        values["SOC"] = m_info_record->soc_power_meter.getValueList();
        nlohmann::ordered_json thread_values;
        for (auto it = m_thread_map.begin(); it != m_thread_map.end(); it++)
        {
            thread_values[it->first] = it->second->time_meter.getValueList();
        }
        values["ThreadTimeInterval"] = thread_values;
        summary["RawValues"] = values;

        return summary;
    }
}