#pragma once

#include <stdio.h>
#include <vector>
#include <cstdint>

using namespace std;

namespace SLAM_Benchmark
{
    template <class T>
    class
        Meter
    {
    private:
        vector<T> m_list;
        T m_min;
        T m_max;
        T m_sum;

    public:
        Meter() : m_list({}), m_min(0), m_max(0), m_sum(0) {}

        T getMin();

        T getMax();

        T getSum();

        T getMean();

        T getStd();

        void update(const T val);
    };
}