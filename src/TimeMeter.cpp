#include "TimeMeter.h"

#include <math.h>

namespace SLAM_Benchmark
{
    void TimeMeter::update(const uint64_t time)
    {
        m_time_list.push_back(time);
        m_time_min = std::min(m_time_min, time);
        m_time_max = std::max(m_time_max, time);
        m_time_sum += time;
    }

    uint64_t TimeMeter::getMin()
    {
        return m_time_min;
    }

    uint64_t TimeMeter::getMax()
    {
        return m_time_max;
    }

    uint64_t TimeMeter::getSum()
    {
        return m_time_sum;
    }

    uint64_t TimeMeter::getMean()
    {
        return m_time_sum / m_time_list.size();
    }

    uint64_t TimeMeter::getStd()
    {
        uint64_t var = 0;
        uint64_t mean = getMean();
        for (uint64_t interval : m_time_list)
        {
            var += (interval - mean) * (interval - mean);
        }
        var /= m_time_list.size();
        return sqrt(var);
    }

}