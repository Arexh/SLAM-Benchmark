#include "SystemRecorder.h"
#include "Utility.h"

#include <thread>
#include <future>
#include <iostream>

SLAM_Benchmark::ThreadRecorder* anotherSimpleTask()
{
    SLAM_Benchmark::ThreadRecorder *record = new SLAM_Benchmark::ThreadRecorder("simple_thread");
    record->recordThreadCreate();
    for (int i = 0; i < 3; i++) {
        record->recordThreadProcessStart();
        SLAM_Benchmark::Utility::simpleComputationTask(1000000000);
        record->recordThreadProcessStop();
    }
    record->recordThreadDestory();
    return record;
}

int main()
{
    SLAM_Benchmark::Utility::thread_flag = true;
    SLAM_Benchmark::SystemRecorder system_info_recorder(SLAM_Benchmark::SystemName::ORB_SLAM2);
    system_info_recorder.recordSystemStart();
    std::promise<SLAM_Benchmark::ThreadRecorder> p;
    std::future<SLAM_Benchmark::ThreadRecorder*> ret = std::async(&anotherSimpleTask);
    SLAM_Benchmark::ThreadRecorder* record = ret.get();
    SLAM_Benchmark::Utility::thread_flag = false;
    system_info_recorder.addThreadRecord(record);
    system_info_recorder.recordSystemStop();
    std::cout << system_info_recorder.summary() << std::endl;
    return 0;
}