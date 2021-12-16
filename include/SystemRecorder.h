#pragma once

#include "SystemInfoManager.h"
#include "ThreadRecorder.h"
#include "Meter.h"
#include "Enums.h"

#include <nlohmann/json.hpp>
#include <stdio.h>
#include <string>
#include <vector>
#include <map>

using namespace std;

namespace SLAM_Benchmark
{
    class SystemRecorder
    {
    private:
        static SystemRecorder *m_system_recorder;
        uint64_t m_start_time;
        uint64_t m_end_time;
        SystemName m_system_name;
        vector<string> m_thread_insert_order;
        map<string, ThreadRecorder *> m_thread_map;
        struct SystemInfoRecord *m_info_record;
        ThreadRecorder *m_publish_record = NULL;
        nlohmann::ordered_json m_summary;

    public:
        static SystemRecorder *getInstance(SystemName system_name)
        {
            if (NULL == m_system_recorder)
            {
                m_system_recorder = new SystemRecorder(system_name);
            }
            return m_system_recorder;
        }

        static void releaseInstance()
        {
            if (NULL != m_system_recorder)
            {
                delete m_system_recorder;
                m_system_recorder = NULL;
            }
        }

        SystemRecorder(SystemName system_name) : m_start_time(0),
                                                 m_end_time(0),
                                                 m_system_name(system_name),
                                                 m_thread_map({}),
                                                 m_info_record(),
                                                 m_publish_record() {}

        ~SystemRecorder();

        void recordSystemStart();

        void recordSystemStop();

        void addThreadRecord(ThreadRecorder *thread_recorder);

        ThreadRecorder *getThreadRecorder(const string &thread_name);

        void addPublishRecord(ThreadRecorder *publish_record);

        nlohmann::ordered_json summary();
    };
}