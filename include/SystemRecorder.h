#pragma once

#include "SystemInfoManager.h"
#include "Meter.h"

#include <stdio.h>
#include <string>
#include <vector>
#include <map>

using namespace std;

namespace SLAM_Benchmark
{
    enum SystemName
    {
        ORB_SLAM2,
        ORB_SLAM3,
        VINS_MONO
    };

    struct SystemThread
    {
        string thread_name;
        uint64_t start_time;
        uint64_t end_time;
        uint64_t temp_time;
        uint64_t thread_time;
        Meter<uint64_t> time_meter;
    };

    class SystemRecorder
    {
    private:
        uint64_t m_start_time;
        uint64_t m_end_time;
        SystemName m_system_name;
        map<string, SystemThread> m_thread_map;
        struct SystemInfoRecord *m_info_record;

    public:
        SystemRecorder(SystemName system_name) : m_system_name(system_name),
                                                 m_start_time(0),
                                                 m_end_time(0),
                                                 m_thread_map({}),
                                                 m_info_record({}) {}

        ~SystemRecorder();

        void registerThread(const string thread_name);

        void recordSystemStart();

        void recordSystemStop();

        void recordThreadCreate(const string thread_name);

        void recordThreadDestory(const string thread_name);

        void recordThreadProcessStart(const string thread_name);

        void recordThreadProcessStop(const string thread_name);

        
    };
}