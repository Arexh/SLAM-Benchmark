#pragma once

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
        Meter<uint64_t> time_meter;
    };

    class SystemRecorder
    {
    private:
        uint64_t start_time;
        SystemName m_system_name;
        map<string, SystemThread> m_thread_map;

    public:
        SystemRecorder(SystemName system_name) : m_system_name(system_name) {}

        ~SystemRecorder();

        void registerThread(const string thread_name);

        void startSystemRecord();

        void stopSystemRecord();

        void startThreadRecord(const string thread_name);

        void stopThreadRecord(const string thread_name);
    };

}