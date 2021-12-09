#pragma once

#include "TimeMeter.h"

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
        TimeMeter time_meter;
    };

    class SLAMSystem
    {
    private:
        SystemName m_system_name;
        map<string, SystemThread> m_thread_map;

    public:
        SLAMSystem(SystemName system_name) : m_system_name(system_name) {}

        ~SLAMSystem();

        void registerThread(const string thread_name);

        void startThreadRecord(const string thread_name);

        void stopThreadRecord(const string thread_name);
    };

}