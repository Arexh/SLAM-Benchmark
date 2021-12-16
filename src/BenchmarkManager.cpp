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
        ORB_SLAM2::System *slam_system = createSystem(system_name, dataset_name, false);
        // create and start the recorder
        SLAM_Benchmark::SystemRecorder *info_recorder = SystemRecorder::getInstance(system_name);
        info_recorder->recordSystemStart();

        SLAM_Benchmark::ThreadRecorder *publish_recorder = new SLAM_Benchmark::ThreadRecorder("Publish");

        vector<double> time_stamp = dataset_loader->getTimestamp();
        int image_num = dataset_loader->getSize();

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

            publish_recorder->recordThreadProcessStart();

            // Pass the image to the SLAM system
            slam_system->TrackMonocular(image, tframe);

            publish_recorder->recordThreadProcessStop();
        }

        // Stop all threads
        slam_system->Shutdown();
        info_recorder->recordSystemStop();
        info_recorder->addPublishRecord(publish_recorder);

        cout << info_recorder->summary() << endl;

        std::ofstream o("orb_slam2.json");
        o << std::setw(4) << info_recorder->summary() << std::endl;
        o.close();
        cout << "Save summary to orb_slam2.json" << endl;

        delete info_recorder;
        delete dataset_loader;
        delete slam_system;
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