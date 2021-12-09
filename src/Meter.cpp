#include "Meter.h"

#include <math.h>

namespace SLAM_Benchmark
{
    template <class T>
    void Meter<T>::update(const T val)
    {
        m_list.push_back(val);
        m_min = std::min(m_min, val);
        m_max = std::max(m_max, val);
        m_sum += val;
    }

    template <class T>
    T Meter<T>::getMin()
    {
        return m_min;
    }

    template <class T>
    T Meter<T>::getMax()
    {
        return m_max;
    }

    template <class T>
    T Meter<T>::getSum()
    {
        return m_sum;
    }

    template <class T>
    T Meter<T>::getMean()
    {
        return m_sum / m_list.size();
    }

    template <class T>
    T Meter<T>::getStd()
    {
        uint64_t var = 0;
        uint64_t mean = getMean();
        for (uint64_t interval : m_list)
        {
            var += (interval - mean) * (interval - mean);
        }
        var /= m_list.size();
        return sqrt(var);
    }

}