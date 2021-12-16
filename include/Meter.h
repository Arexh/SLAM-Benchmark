#pragma once

#include <stdio.h>
#include <vector>
#include <cstdint>
#include <math.h>
#include <nlohmann/json.hpp>

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

        vector<T> getValueList();

        void update(const T val);

        nlohmann::ordered_json summaryStatistics(const string &prefix = "");
    };

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
        if (m_list.size() == 0) return -1;
        return m_min;
    }

    template <class T>
    T Meter<T>::getMax()
    {
        if (m_list.size() == 0) return -1;
        return m_max;
    }

    template <class T>
    T Meter<T>::getSum()
    {
        if (m_list.size() == 0) return -1;
        return m_sum;
    }

    template <class T>
    T Meter<T>::getMean()
    {
        if (m_list.size() == 0) return -1;
        return m_sum / m_list.size();
    }

    template <class T>
    T Meter<T>::getStd()
    {
        if (m_list.size() == 0) return -1;
        uint64_t var = 0;
        uint64_t mean = getMean();
        for (uint64_t interval : m_list)
        {
            var += (interval - mean) * (interval - mean);
        }
        var /= m_list.size();
        return sqrt(var);
    }

    template <class T>
    vector<T> Meter<T>::getValueList()
    {
        return m_list;
    }

    template <class T>
    nlohmann::ordered_json Meter<T>::summaryStatistics(const string &prefix)
    {
        if (m_list.size() == 0)
        {
            return {
                {"min", 0},
                {"max", 0},
                {"avg", 0},
                {"std", 0},
                {"count", 0}};
        }
        return {
            {"min", getMin()},
            {"max", getMax()},
            {"avg", getMean()},
            {"std", getStd()},
            {"count", m_list.size()}};
    }
}