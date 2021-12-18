#pragma once

#include "Meter.h"
#include <string>

namespace SLAM_Benchmark
{
    class ThreadRecorder
    {
    private:
        friend class SystemRecorder;
        std::string m_thread_name;
        uint64_t m_start_time;
        uint64_t m_end_time;
        uint64_t m_temp_time;
        uint64_t m_thread_time;
        Meter<uint64_t> m_time_meter;
        vector<string> m_insert_order;
        map<string, Meter<uint64_t>*> m_subprocess_meter;
        map<string, uint64_t> m_subprocess_time;

    public:
        ThreadRecorder(const string &thread_name) : m_thread_name(thread_name),
                                                    m_start_time(0),
                                                    m_end_time(0),
                                                    m_temp_time(0),
                                                    m_thread_time(0),
                                                    m_time_meter(),m_subprocess_meter({}),
                                                    m_subprocess_time({}) {}

        void recordThreadCreate();

        void recordThreadDestory();

        void recordThreadProcessStart();

        void recordThreadProcessStop();

        void recordSubprocessStart(const string &process_name);

        void recordSubprocessStop(const string &process_name);

        void createSubprocess(const string &process_name);

        nlohmann::ordered_json summary();

        nlohmann::ordered_json rawValue();
    };
}