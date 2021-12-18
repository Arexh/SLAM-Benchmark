#include "ThreadRecorder.h"
#include <chrono>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>

#include "Utility.h"

namespace SLAM_Benchmark
{
    /*
        struct timespec {
            time_t   tv_sec;        / seconds
            long     tv_nsec;       // nanoseconds
        };
    */
    struct timespec getThreadTime();

    void ThreadRecorder::recordThreadCreate()
    {
        m_start_time = Utility::getCurrentNanosecond();
    }

    void ThreadRecorder::recordThreadDestory()
    {
        m_end_time = Utility::getCurrentNanosecond();
        m_thread_time = getThreadTime().tv_nsec;
    }

    void ThreadRecorder::recordThreadProcessStart()
    {
        m_temp_time = Utility::getCurrentNanosecond();
    }

    void ThreadRecorder::recordThreadProcessStop()
    {
        m_time_meter.update(Utility::getCurrentNanosecond() - m_temp_time);
    }

    void ThreadRecorder::createSubprocess(const string &process_name)
    {
        m_subprocess_meter[process_name] = new Meter<uint64_t>();
        m_insert_order.push_back(process_name);
    }

    void ThreadRecorder::recordSubprocessStart(const string &process_name)
    {
        m_subprocess_time[process_name] = Utility::getCurrentNanosecond();
    }

    void ThreadRecorder::recordSubprocessStop(const string &process_name)
    {
        m_subprocess_meter[process_name]->update(Utility::getCurrentNanosecond() - m_subprocess_time[process_name]);
    }

    nlohmann::ordered_json ThreadRecorder::summary()
    {
        nlohmann::ordered_json summary = {{"StartTime", m_start_time},
                                          {"EndTime", m_end_time},
                                          {"Interval", m_end_time - m_start_time},
                                          {"ThreadTime", m_thread_time},
                                          {"ProcessTime", m_time_meter.summaryStatistics()}};
        nlohmann::ordered_json subprocess_summary = {};
        for (unsigned int i = 0; i < m_insert_order.size(); i++)
        {
            subprocess_summary[m_insert_order[i]] = m_subprocess_meter[m_insert_order[i]]->summaryStatistics();
        }
        summary["Subprocess"] = subprocess_summary;
        return summary;
    }

    nlohmann::ordered_json ThreadRecorder::rawValue()
    {
        nlohmann::ordered_json raw_value = {};
        raw_value["ProcessTime"] = m_time_meter.getValueList();
        for (unsigned int i = 0; i < m_insert_order.size(); i++)
        {
            raw_value[m_insert_order[i]] = m_subprocess_meter[m_insert_order[i]]->getValueList();
        }
        return raw_value;
    }

    // copy from: https://stackoverflow.com/questions/44916362/how-can-i-measure-cpu-time-of-a-specific-set-of-threads
    struct timespec getThreadTime()
    {
        struct timespec currTime;
        clockid_t threadClockId;
        //! Get thread clock Id
        pthread_getcpuclockid(pthread_self(), &threadClockId);
        //! Using thread clock Id get the clock time
        clock_gettime(threadClockId, &currTime);
        return currTime;
    }
}