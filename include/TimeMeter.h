#pragma once

#include <stdio.h>
#include <vector>
#include <cstdint>

using namespace std;

namespace SLAM_Benchmark
{
    class TimeMeter
    {
    private:
        vector<uint64_t> m_time_list;
        uint64_t m_time_min;
        uint64_t m_time_max;
        uint64_t m_time_sum;

    public:
        TimeMeter() : m_time_list({}), m_time_min(0), m_time_max(0), m_time_sum(0) {}

        void update(const uint64_t time);

        uint64_t getMin();

        uint64_t getMax();

        uint64_t getSum();

        uint64_t getMean();

        uint64_t getStd();
    };
}