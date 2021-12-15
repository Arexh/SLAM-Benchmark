#pragma once

#include <string>
#include <stdlib.h>
#include <fstream>
#include <chrono>
#include <cmath>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace SLAM_Benchmark
{
    class Utility
    {
    public:
        static std::atomic_bool thread_flag;

        static uint64_t getCurrentMillisecond();

        static bool checkIfFileExists(const std::string &file_name);

        static int readFileContentInt(const std::string &file_name);

        static void timedTask(const uint32_t interval, const std::function<void()> callback);

        static double simpleComputationTask(unsigned long long loops);
        
        template <class T>
        static T roundDecimal(T number, int decimalVal);

        static double calculateFPS(uint64_t interval);
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

    template <class T>
    T Utility::roundDecimal(T number, int decimalVal)
    {
        double powerOfTen = pow(10, decimalVal);
        T value = (long)(number * powerOfTen + .5);
        return (T) value / powerOfTen;
    }

    inline double Utility::calculateFPS(uint64_t interval)
    {
        return 1000.0 / interval;
    }
}