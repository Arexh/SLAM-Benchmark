#include "SystemRecorder.h"
#include "Utility.h"

#include <thread>
#include <future>
#include <iostream>
#include <bits/stdc++.h>

std::pair<SLAM_Benchmark::ThreadRecorder*, SLAM_Benchmark::ThreadRecorder*> anotherSimpleTask()
{
    SLAM_Benchmark::ThreadRecorder *record = new SLAM_Benchmark::ThreadRecorder("simple_thread");
    SLAM_Benchmark::ThreadRecorder *publish_record = new SLAM_Benchmark::ThreadRecorder("publish");
    publish_record->recordThreadCreate();
    record->recordThreadCreate();
    for (int i = 0; i < 3; i++) {
        record->recordThreadProcessStart();
        for (int j = 0; j < 3; j++) {
            publish_record->recordThreadProcessStart();
            std::cout << "Mock publish!" << std::endl;
            usleep(1000000);
            publish_record->recordThreadProcessStop();
        }
        SLAM_Benchmark::Utility::simpleComputationTask(1000000000);
        record->recordThreadProcessStop();
    }
    record->recordThreadDestory();
    publish_record->recordThreadDestory();
    return std::make_pair(record, publish_record);
}

int main()
{
    SLAM_Benchmark::Utility::thread_flag = true;
    SLAM_Benchmark::SystemRecorder system_info_recorder(SLAM_Benchmark::SystemName::ORB_SLAM2);
    system_info_recorder.recordSystemStart();
    std::promise<SLAM_Benchmark::ThreadRecorder> p;
    std::future<std::pair<SLAM_Benchmark::ThreadRecorder*, SLAM_Benchmark::ThreadRecorder*>> ret = std::async(&anotherSimpleTask);
    std::pair<SLAM_Benchmark::ThreadRecorder*, SLAM_Benchmark::ThreadRecorder*> record = ret.get();
    SLAM_Benchmark::Utility::thread_flag = false;
    system_info_recorder.addThreadRecord(record.first);
    system_info_recorder.addPublishRecord(record.second);
    system_info_recorder.recordSystemStop();
    std::cout << system_info_recorder.summary() << std::endl;
    return 0;
}