#pragma once
#include <chrono>
#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <stdlib.h>
#include <fstream>
#include <chrono>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace SLAM_Benchmark
{
    class Utility
    {
    public:
        static uint64_t getCurrentMillisecond();

        static bool checkIfFileExists(const std::string &file_name);

        static int readFileContentInt(const std::string &file_name);

        static void timedTask(const uint32_t interval,
                              const bool &flag,
                              const std::function<void()> callback);

        static double simpleComputationTask(unsigned long long loops);
    };

    inline uint64_t Utility::getCurrentMillisecond()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }

    inline bool Utility::checkIfFileExists(const std::string &file_name)
    {
        struct stat buffer;
        return (stat(file_name.c_str(), &buffer) == 0);
    }

    inline int Utility::readFileContentInt(const std::string &file_name)
    {
        std::ifstream file_stream;
        std::string str;
        file_stream.open(file_name);
        file_stream >> str;
        file_stream.close();
        return atoi(str.c_str());
    }
}