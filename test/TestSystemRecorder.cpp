#include "SystemRecorder.h"
#include "Utility.h"

#include <thread>
#include <future>
#include <iostream>
#include <bits/stdc++.h>
#include <nlohmann/json.hpp>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

void anotherSimpleTask(SLAM_Benchmark::ThreadRecorder *record, SLAM_Benchmark::ThreadRecorder *publish_record)
{
    publish_record->recordThreadCreate();
    record->recordThreadCreate();
    for (int i = 0; i < 3; i++)
    {
        record->recordThreadProcessStart();
        for (int j = 0; j < 3; j++)
        {
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
}

int main()
{
    SLAM_Benchmark::Utility::thread_flag = true;
    SLAM_Benchmark::SystemRecorder system_info_recorder(SLAM_Benchmark::SystemName::ORB_SLAM2);
    system_info_recorder.recordSystemStart();
    SLAM_Benchmark::ThreadRecorder *record = new SLAM_Benchmark::ThreadRecorder("simple_thread");
    SLAM_Benchmark::ThreadRecorder *publish_record = new SLAM_Benchmark::ThreadRecorder("publish");
    std::thread thread([&] {anotherSimpleTask(record, publish_record);});
    cout << "HJEREFEFE" << endl;
    thread.join();
    SLAM_Benchmark::Utility::thread_flag = false;
    system_info_recorder.addThreadRecord(record);
    system_info_recorder.addPublishRecord(publish_record);
    system_info_recorder.recordSystemStop();

    nlohmann::ordered_json summary = system_info_recorder.summary();
    std::cout << summary << std::endl;
    std::ofstream o("summary.json");

    o << std::setw(4) << summary << std::endl;
    o.close();

    vector<double> y = summary["RawValues"]["PublishTime"];
    vector<int> x = {};
    for (long unsigned int i = 0; i < y.size(); i++)
        x.push_back(i + 1);
    plt::named_plot("Publish Time", x, y);
    plt::legend();
    plt::xlabel("Image");
    plt::ylabel("Millisecond");
    plt::save("test.png", 400);

    return 0;
}