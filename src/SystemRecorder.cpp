#include "SystemRecorder.h"
#include "Utility.h"

namespace SLAM_Benchmark
{
    SystemRecorder *SystemRecorder::m_system_recorder = 0;

    SystemRecorder::~SystemRecorder()
    {
        delete m_info_record;
        delete m_publish_record;

        for (auto it = m_thread_map.begin(); it != m_thread_map.end(); it++)
        {
            delete it->second;
        }
    }

    void SystemRecorder::recordSystemStart()
    {
        m_start_time = Utility::getCurrentNanosecond();
        SystemInfoManager::startMonitor();
    }

    void SystemRecorder::recordSystemStop()
    {
        m_end_time = Utility::getCurrentNanosecond();
        m_info_record = SystemInfoManager::stopMonitor();
    }

    void SystemRecorder::addThreadRecord(ThreadRecorder *thread_recorder)
    {
        m_thread_map[thread_recorder->m_thread_name] = thread_recorder;
        m_thread_insert_order.push_back(thread_recorder->m_thread_name);
    }

    ThreadRecorder* SystemRecorder::getThreadRecorder(const string &thread_name)
    {
        return m_thread_map[thread_name];
    }

    void SystemRecorder::addPublishRecord(ThreadRecorder *publish_recorder)
    {
        m_publish_record = publish_recorder;
    }

    ThreadRecorder* SystemRecorder::getPublishRecord()
    {
        return m_publish_record;
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
            {"AvgFPS", 1000000000.0 / m_publish_record->m_time_meter.getMean()}};

        if (m_publish_record != NULL)
            summary["PublishTime"] = m_publish_record->m_time_meter.summaryStatistics();

        if (SystemInfoManager::isCPUPowerAvailable())
        {
            summary["CPUPower"] = m_info_record->cpu_power_meter.summaryStatistics();
        }
        else
        {
            summary["CPUPower"] = "Unavailable";
        }
        if (SystemInfoManager::isGPUPowerAvailable())
        {
            summary["GPUPower"] = m_info_record->gpu_power_meter.summaryStatistics();
        }
        else
        {
            summary["GPUPower"] = "Unavailable";
        }
        if (SystemInfoManager::isSOCPowerAvailable())
        {
            summary["TotalPower"] = m_info_record->soc_power_meter.summaryStatistics();
        }
        else
        {
            summary["TotalPower"] = "Unavailable";
        }

        nlohmann::ordered_json thread_info;
        for (unsigned int i = 0; i < m_thread_insert_order.size(); i++)
        {
            ThreadRecorder *record = m_thread_map[m_thread_insert_order[i]];
            thread_info[m_thread_insert_order[i]] = record->summary();
        }
        summary["Threads"] = thread_info;

        nlohmann::ordered_json values;
        values["PublishTime"] = m_publish_record->m_time_meter.getValueList();

        values["CPU"] = m_info_record->cpu_percent_meter.getValueList();
        values["VirtualMemory"] = m_info_record->virtual_memory_meter.getValueList();
        values["PhysicalMemory"] = m_info_record->physical_memory_meter.getValueList();
        values["CPUPower"] = m_info_record->cpu_power_meter.getValueList();
        values["GPUPower"] = m_info_record->gpu_power_meter.getValueList();
        values["TotalPower"] = m_info_record->soc_power_meter.getValueList();
        nlohmann::ordered_json thread_values;
        for (auto it = m_thread_map.begin(); it != m_thread_map.end(); it++)
        {
            thread_values[it->first] = it->second->rawValue();
        }
        values["Threads"] = thread_values;
        summary["RawValues"] = values;

        return summary;
    }
}