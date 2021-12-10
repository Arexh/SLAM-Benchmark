#pragma once

#include "SystemInfoManager.h"
#include "ThreadRecorder.h"
#include "EnumToString.h"
#include "Meter.h"

#include <nlohmann/json.hpp>
#include <stdio.h>
#include <string>
#include <vector>
#include <map>

using namespace std;

namespace SLAM_Benchmark
{
    DEFINE_ENUM_WITH_STRING_CONVERSIONS(SystemName, (ORB_SLAM2)(ORB_SLAM3)(VINS_MONO));

    class SystemRecorder
    {
    private:
        uint64_t m_start_time;
        uint64_t m_end_time;
        SystemName m_system_name;
        map<string, ThreadRecorder*> m_thread_map;
        struct SystemInfoRecord *m_info_record;
        ThreadRecorder* m_publish_record;

    public:
        SystemRecorder(SystemName system_name) : m_start_time(0),
                                                 m_end_time(0),
                                                 m_system_name(system_name),
                                                 m_thread_map({}),
                                                 m_info_record(),
                                                 m_publish_record() {}

        ~SystemRecorder();

        void recordSystemStart();

        void recordSystemStop();

        void addThreadRecord(ThreadRecorder* thread_recorder);

        void addPublishRecord(ThreadRecorder* publish_record);

        nlohmann::ordered_json summary();
    };
}