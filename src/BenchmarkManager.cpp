#include "BenchmarkManager.h"
#include "ORB_SLAM2_Injection/System.h"
#include "ORB_SLAM2_Injection/LoopClosing.h"
#include "ORB_SLAM2_Injection/LocalMapping.h"
#include "ORB_SLAM3_Injection/System.h"
#include "ORB_SLAM3_Injection/LoopClosing.h"
#include "ORB_SLAM3_Injection/LocalMapping.h"
#include "Utility.h"

#include "ORB_SLAM3_detailed_comments/include/System.h"

#include <iostream>
#include <string>
#include <unistd.h>

namespace SLAM_Benchmark
{
    const std::string BenchmarkManager::ORB_SLAM2_VOC_PATH = _ORB_SLAM2_VOC_PATH;

    const std::string BenchmarkManager::ORB_SLAM3_VOC_PATH = _ORB_SLAM3_VOC_PATH;

    const std::map<DatasetName, std::string> BenchmarkManager::ORB_SLAM2_SETTING_MAP = {
        {DatasetName::TUM, _ORB_SLAM2_TUM_SETTING_PATH}};

    const std::map<DatasetName, std::string> BenchmarkManager::ORB_SLAM3_SETTING_MAP = {
        {DatasetName::TUM, _ORB_SLAM3_TUM_SETTING_PATH}};

    void BenchmarkManager::benchmark_ORB_SLAM2(DatasetName dataset_name, const std::string dataset_path, bool use_viewr)
    {
        /* init recorders */
        SLAM_Benchmark::SystemRecorder *system_recorder = SLAM_Benchmark::SystemRecorder::getInstance(SLAM_Benchmark::SystemName::ORB_SLAM2);
        SLAM_Benchmark::ThreadRecorder *tracking_recorder = new SLAM_Benchmark::ThreadRecorder("Tracking");
        SLAM_Benchmark::ThreadRecorder *local_mapper_recorder = new SLAM_Benchmark::ThreadRecorder("LocalMapping");
        SLAM_Benchmark::ThreadRecorder *loop_closing_recorder = new SLAM_Benchmark::ThreadRecorder("LoopClosing");
        SLAM_Benchmark::ThreadRecorder *bundle_adjustment_recorder = new SLAM_Benchmark::ThreadRecorder("BundleAdjustment");
        system_recorder->addThreadRecord(tracking_recorder);
        system_recorder->addThreadRecord(local_mapper_recorder);
        system_recorder->addThreadRecord(loop_closing_recorder);
        system_recorder->addThreadRecord(bundle_adjustment_recorder);
        tracking_recorder->createSubprocess("ORBExtraction");
        tracking_recorder->createSubprocess("Track");
        local_mapper_recorder->createSubprocess("ProcessNewKeyFrame");
        local_mapper_recorder->createSubprocess("MapPointCulling");
        local_mapper_recorder->createSubprocess("CreateNewMapPoints");
        local_mapper_recorder->createSubprocess("SearchInNeighbors");
        local_mapper_recorder->createSubprocess("LocalBundleAdjustment");
        local_mapper_recorder->createSubprocess("KeyFrameCulling");
        loop_closing_recorder->createSubprocess("DetectLoop");
        loop_closing_recorder->createSubprocess("ComputeSim3");
        loop_closing_recorder->createSubprocess("SearchAndFuse");
        loop_closing_recorder->createSubprocess("OptimizeEssentialGraph");
        bundle_adjustment_recorder->createSubprocess("FullBundleAdjustment");
        bundle_adjustment_recorder->createSubprocess("MapUpdate");
        /* init recorders */

        system_recorder->recordSystemStart();
        DatasetLoader *dataset_loader = createDatasetLoader(dataset_name, dataset_path);
        ORB_SLAM2::System *slam_system = createSystem(SLAM_Benchmark::SystemName::ORB_SLAM2, dataset_name, use_viewr);
        // create and start the recorder

        SLAM_Benchmark::ThreadRecorder *publish_recorder = new SLAM_Benchmark::ThreadRecorder("Publish");

        vector<double> time_stamp = dataset_loader->getTimestamp();
        int image_num = dataset_loader->getSize();

        cout << endl
             << "-------" << endl;
        cout << "Start processing sequence ..." << endl;
        cout << "Images in the sequence: " << image_num << endl
             << endl;

        tracking_recorder->recordThreadCreate();
        cv::Mat image;
        for (int i = 0; i < image_num; i++)
        {
            tracking_recorder->recordThreadProcessStart();
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

            tracking_recorder->recordThreadProcessStop();
        }

        // Stop all threads
        slam_system->Shutdown();
        system_recorder->recordSystemStop();
        system_recorder->addPublishRecord(publish_recorder);
        tracking_recorder->recordThreadDestory();

        cout << system_recorder->summary() << endl;

        std::ofstream o("orb_slam2.json");
        o << std::setw(4) << system_recorder->summary() << std::endl;
        o.close();
        cout << "Save summary to orb_slam2.json" << endl;

        delete system_recorder;
        delete dataset_loader;
        delete slam_system;
    }

    void BenchmarkManager::benchmark_ORB_SLAM3(DatasetName dataset_name, const std::string dataset_path, bool use_viewr)
    {
        /* init recorders */
        SLAM_Benchmark::SystemRecorder *system_recorder = SLAM_Benchmark::SystemRecorder::getInstance(SLAM_Benchmark::SystemName::ORB_SLAM3);
        SLAM_Benchmark::ThreadRecorder *tracking_recorder = new SLAM_Benchmark::ThreadRecorder("Tracking");
        SLAM_Benchmark::ThreadRecorder *local_mapper_recorder = new SLAM_Benchmark::ThreadRecorder("LocalMapping");
        SLAM_Benchmark::ThreadRecorder *loop_closing_recorder = new SLAM_Benchmark::ThreadRecorder("LoopClosing");
        SLAM_Benchmark::ThreadRecorder *bundle_adjustment_recorder = new SLAM_Benchmark::ThreadRecorder("BundleAdjustment");
        system_recorder->addThreadRecord(tracking_recorder);
        system_recorder->addThreadRecord(local_mapper_recorder);
        system_recorder->addThreadRecord(loop_closing_recorder);
        system_recorder->addThreadRecord(bundle_adjustment_recorder);
        tracking_recorder->createSubprocess("ORBExtraction");
        tracking_recorder->createSubprocess("Track");
        local_mapper_recorder->createSubprocess("ProcessNewKeyFrame");
        local_mapper_recorder->createSubprocess("MapPointCulling");
        local_mapper_recorder->createSubprocess("CreateNewMapPoints");
        local_mapper_recorder->createSubprocess("SearchInNeighbors");
        local_mapper_recorder->createSubprocess("LocalBundleAdjustment");
        local_mapper_recorder->createSubprocess("KeyFrameCulling");
        loop_closing_recorder->createSubprocess("DetectLoop");
        loop_closing_recorder->createSubprocess("ComputeSim3");
        loop_closing_recorder->createSubprocess("SearchAndFuse");
        loop_closing_recorder->createSubprocess("OptimizeEssentialGraph");
        bundle_adjustment_recorder->createSubprocess("FullBundleAdjustment");
        bundle_adjustment_recorder->createSubprocess("MapUpdate");
        /* init recorders */

        system_recorder->recordSystemStart();
        DatasetLoader *dataset_loader = createDatasetLoader(dataset_name, dataset_path);
        SLAM_Benchmark::ORB_SLAM3_Inject::System *slam_system = new SLAM_Benchmark::ORB_SLAM3_Inject::System(ORB_SLAM3_VOC_PATH,
                                                                                                             ORB_SLAM3_SETTING_MAP.at(dataset_name),
                                                                                                             ORB_SLAM3::System::eSensor::MONOCULAR,
                                                                                                             use_viewr);
        // create and start the recorder

        SLAM_Benchmark::ThreadRecorder *publish_recorder = new SLAM_Benchmark::ThreadRecorder("Publish");

        vector<double> time_stamp = dataset_loader->getTimestamp();
        int image_num = dataset_loader->getSize();

        cout << endl
             << "-------" << endl;
        cout << "Start processing sequence ..." << endl;
        cout << "Images in the sequence: " << image_num << endl
             << endl;

        tracking_recorder->recordThreadCreate();
        cv::Mat image;
        for (int i = 0; i < image_num; i++)
        {
            tracking_recorder->recordThreadProcessStart();
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

            tracking_recorder->recordThreadProcessStop();
        }

        // Stop all threads
        slam_system->Shutdown();
        system_recorder->recordSystemStop();
        system_recorder->addPublishRecord(publish_recorder);
        tracking_recorder->recordThreadDestory();

        cout << system_recorder->summary() << endl;

        std::ofstream o("orb_slam3.json");
        o << std::setw(4) << system_recorder->summary() << std::endl;
        o.close();
        cout << "Save summary to orb_slam3.json" << endl;

        delete system_recorder;
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