#include "BenchmarkManager.h"
#include "ORB_SLAM2_Injection/System.h"
#include "ORB_SLAM2_Injection/LoopClosing.h"
#include "ORB_SLAM2_Injection/LocalMapping.h"
#include "Utility.h"

#include <iostream>
#include <string>
#include <unistd.h>

namespace SLAM_Benchmark
{
    const std::string BenchmarkManager::ORB_SLAM2_VOC_PATH = _ORB_SLAM2_VOC_PATH;

    const std::map<DatasetName, std::string> BenchmarkManager::ORB_SLAM2_SETTING_MAP = {
        {DatasetName::TUM, _ORB_SLAM2_TUM_SETTING_PATH}};

    void BenchmarkManager::benchmark(SystemName system_name, DatasetName dataset_name, const std::string dataset_path)
    {
        DatasetLoader *dataset_loader = createDatasetLoader(dataset_name, dataset_path);
        ORB_SLAM2::System *slam_system = createSystem(system_name, dataset_name, true);
        // create and start the recorder
        // SLAM_Benchmark::SystemRecorder *info_recorder = SystemRecorder::getInstance(system_name);
        // SLAM_Benchmark::Utility::thread_flag = true;
        // info_recorder->recordSystemStart();

        vector<double> time_stamp = dataset_loader->getTimestamp();
        int image_num = dataset_loader->getSize();

        vector<float> time_track;
        time_track.resize(image_num);

        cout << endl
             << "-------" << endl;
        cout << "Start processing sequence ..." << endl;
        cout << "Images in the sequence: " << image_num << endl
             << endl;

        cv::Mat image;
        for (int i = 0; i < image_num; i++)
        {
            // Read image from file
            double tframe = time_stamp[i];

            if (!dataset_loader->loadImage(i, image))
            {
                return;
            }

            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            // Pass the image to the SLAM system
            slam_system->TrackMonocular(image, tframe);

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

            double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

            time_track[i] = ttrack;

            // Wait to load the next frame
            double T = 0;
            if (i < image_num - 1)
                T = time_stamp[i + 1] - tframe;
            else if (i > 0)
                T = tframe - time_stamp[i - 1];

            if (ttrack < T)
                usleep((T - ttrack) * 1e6);
        }

        // Stop all threads
        slam_system->Shutdown();
        // info_recorder->recordSystemStop();

        // Tracking time statistics
        sort(time_stamp.begin(), time_stamp.end());
        float totaltime = 0;
        for (int i = 0; i < image_num; i++)
        {
            totaltime += time_track[i];
        }
        cout << "-------" << endl
             << endl;
        cout << "median tracking time: " << time_track[image_num / 2] << endl;
        cout << "mean tracking time: " << totaltime / image_num << endl;
    }

    DatasetLoader *BenchmarkManager::createDatasetLoader(DatasetName dataset_name, const std::string &dataset_path)
    {
        DatasetLoader *dataset_loader;
        switch (dataset_name)
        {
        case DatasetName::TUM:
            dataset_loader = new TUMDatasetLoader(dataset_path);
            break;
        default:
            dataset_loader = new TUMDatasetLoader(dataset_path);
        }
        return dataset_loader;
    }

    ORB_SLAM2::System *BenchmarkManager::createSystem(SystemName system_name, DatasetName dataset_name, bool use_viewer)
    {
        ORB_SLAM2::System *system;
        switch (system_name)
        {
        case SystemName::ORB_SLAM2:
            system = new SLAM_Benchmark::ORB_SLAM2_Inject::System(ORB_SLAM2_VOC_PATH,
                                                                  ORB_SLAM2_SETTING_MAP.at(dataset_name),
                                                                  ORB_SLAM2::System::eSensor::MONOCULAR,
                                                                  use_viewer);
            break;
        default:
            system = new SLAM_Benchmark::ORB_SLAM2_Inject::System(ORB_SLAM2_VOC_PATH,
                                                                  ORB_SLAM2_SETTING_MAP.at(dataset_name),
                                                                  ORB_SLAM2::System::eSensor::MONOCULAR,
                                                                  use_viewer);
        }
        return system;
    }
}