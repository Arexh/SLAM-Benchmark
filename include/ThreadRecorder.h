#pragma once

#include "Meter.h"
#include <string>

namespace SLAM_Benchmark
{
    class ThreadRecorder
    {
    private:
        friend class SystemRecorder;
        std::string thread_name;
        uint64_t start_time;
        uint64_t end_time;
        uint64_t temp_time;
        uint64_t thread_time;
        Meter<uint64_t> time_meter;

    public:
        ThreadRecorder(const string &thread_name) : thread_name(thread_name),
                                                    start_time(0),
                                                    end_time(0),
                                                    temp_time(0),
                                                    thread_time(0),
                                                    time_meter() {}

        void recordThreadCreate();

        void recordThreadDestory();

        void recordThreadProcessStart();

        void recordThreadProcessStop();
    };
}