#include "SystemInfoManager.h"
#include "stdio.h"
#include "matplotlibcpp.h"
#include <chrono>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace SLAM_Benchmark;

std::atomic_bool stop_thread_1;

void monitorCPUUsageTask(uint32_t interval)
{
    SystemInfo::init();
    while (stop_thread_1) {
        boost::asio::io_service io;
        boost::asio::deadline_timer t(io, boost::posix_time::milliseconds(interval));
        t.async_wait([](const boost::system::error_code&){
            std::cout << "CPU usage: " << SystemInfo::getCurrentProcessCPUPercent() << ", "
            << "Physical Memory usage: " << SystemInfo::getCurrentProcessPhysicalMemoryUsed() << ", "
            << "Virtual Memory usage: " << SystemInfo::getCurrentProcessVirtualMemoryUsed()
            << std::endl;
        });
        io.run();
    }
}

int main()
{
    stop_thread_1 = true;
    std::vector<std::thread> threads;
    threads.push_back(std::thread(SystemInfoManager::simpleComputationTask, 10000000000));
    threads.push_back(std::thread(monitorCPUUsageTask, 1000));
    for (auto &th : threads) {
        th.join();
        stop_thread_1 = false;
    }
    std::cout << SystemInfo::getCurrentProcessCPUPercent() << std::endl;
    return 0;
}