#pragma once

#include "ORB_SLAM3_detailed_comments/include/System.h"

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Tracking.h"

namespace SLAM_Benchmark
{
    namespace ORB_SLAM3_Inject
    {
        class System : public ORB_SLAM3::System
        {
        public:
            System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const string &strSequence = std::string(), const string &strLoadingFile = std::string()) : ORB_SLAM3::System()
            {
                mSensor = sensor;
                mpViewer = static_cast<ORB_SLAM3::Viewer*>(NULL);
                mbReset = false;
                mbActivateLocalizationMode = false;
                mbDeactivateLocalizationMode = false;
                // Output welcome message
                cout << endl
                     << "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl
                     << "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl
                     << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
                     << "This is free software, and you are welcome to redistribute it" << endl
                     << "under certain conditions. See LICENSE.txt." << endl
                     << endl;

                cout << "Input sensor was set to: ";
                // Step 1 输出当前传感器类型
                if (mSensor == MONOCULAR)
                    cout << "Monocular" << endl; //单目
                else if (mSensor == STEREO)
                    cout << "Stereo" << endl; //双目
                else if (mSensor == RGBD)
                    cout << "RGB-D" << endl; //RGBD相机
                else if (mSensor == IMU_MONOCULAR)
                    cout << "Monocular-Inertial" << endl; //单目 + imu
                else if (mSensor == IMU_STEREO)
                    cout << "Stereo-Inertial" << endl; //双目 + imu

                //Check settings file
                // Step 2 读取配置文件
                cv::FileStorage fsSettings(strSettingsFile.c_str(), //将配置文件名转换成为字符串
                                           cv::FileStorage::READ);  //只读
                //如果打开失败，就输出错误信息
                if (!fsSettings.isOpened())
                {
                    cerr << "Failed to open settings file at: " << strSettingsFile << endl;
                    exit(-1);
                }

                // ORBSLAM3新加的多地图管理功能，这里加载Atlas标识符
                bool loadedAtlas = false;

                //----
                //Load ORB Vocabulary
                // Step 3 加载ORB字典
                cout << endl
                     << "Loading ORB Vocabulary. This could take a while..." << endl;

                //建立一个新的ORB字典
                mpVocabulary = new ORB_SLAM3::ORBVocabulary();
                //读取预训练好的ORB字典并返回成功/失败标志
                bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
                //如果加载失败，就输出错误信息
                if (!bVocLoad)
                {
                    cerr << "Wrong path to vocabulary. " << endl;
                    cerr << "Falied to open at: " << strVocFile << endl;
                    exit(-1);
                }
                cout << "Vocabulary loaded!" << endl
                     << endl;

                //Create KeyFrame Database
                // Step 4 创建关键帧数据库
                mpKeyFrameDatabase = new ORB_SLAM3::KeyFrameDatabase(*mpVocabulary);

                //Create the Atlas
                // Step 5 创建多地图，参数0表示初始化关键帧id为0
                mpAtlas = new ORB_SLAM3::Atlas(0);

                if (mSensor == IMU_STEREO || mSensor == IMU_MONOCULAR)
                    // 如果是有imu的传感器类型，设置mbIsInertial = true;以后的跟踪和预积分将和这个标志有关
                    mpAtlas->SetInertialSensor();

                // Step 6 依次创建跟踪、局部建图、闭环、显示线程
                //Create Drawers. These are used by the Viewer
                // 创建用于显示帧和地图的类，由Viewer调用
                mpFrameDrawer = new ORB_SLAM3::FrameDrawer(mpAtlas);
                mpMapDrawer = new ORB_SLAM3::MapDrawer(mpAtlas, strSettingsFile);

                //Initialize the Tracking thread
                //(it will live in the main thread of execution, the one that called this constructor)
                // 创建跟踪线程（主线程）,不会立刻开启,会在对图像和imu预处理后在main主线程种执行
                cout << "Seq. Name: " << strSequence << endl;
                Tracking *mpTrackerInjected = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                                                           mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor, strSequence);
                mpTracker = mpTrackerInjected;

                //Initialize the Local Mapping thread and launch
                //创建并开启local mapping线程
                LocalMapping *mpLocalMapperInjected = new LocalMapping(this, mpAtlas, mSensor == MONOCULAR || mSensor == IMU_MONOCULAR, mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO, strSequence);
                mpLocalMapper = mpLocalMapperInjected;
                mptLocalMapping = new thread(&LocalMapping::Run, mpLocalMapperInjected);

                //设置最远3D地图点的深度值，如果超过阈值，说明可能三角化不太准确，丢弃
                mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
                // ? 这里有个疑问,C++中浮点型跟0比较是否用精确?
                if (mpLocalMapper->mThFarPoints != 0)
                {
                    cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
                    mpLocalMapper->mbFarPoints = true;
                }
                else
                    mpLocalMapper->mbFarPoints = false;

                //Initialize the Loop Closing thread and launch
                // mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR
                // 创建并开启闭环线程
                LoopClosing *mpLoopCloserInjected = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR); // mSensor!=MONOCULAR);
                mpLoopCloser = mpLoopCloserInjected;
                mptLoopClosing = new thread(&LoopClosing::Run, mpLoopCloserInjected);

                //Initialize the Viewer thread and launch
                // 创建并开启显示线程
                if (bUseViewer)
                {
                    mpViewer = new ORB_SLAM3::Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
                    mptViewer = new thread(&ORB_SLAM3::Viewer::Run, mpViewer);
                    mpTracker->SetViewer(mpViewer);
                    mpLoopCloser->mpViewer = mpViewer;
                    mpViewer->both = mpFrameDrawer->both;
                }

                //Set pointers between threads
                // 设置线程间的指针
                mpTracker->SetLocalMapper(mpLocalMapper);
                mpTracker->SetLoopClosing(mpLoopCloser);

                mpLocalMapper->SetTracker(mpTracker);
                mpLocalMapper->SetLoopCloser(mpLoopCloser);

                mpLoopCloser->SetTracker(mpTracker);
                mpLoopCloser->SetLocalMapper(mpLocalMapper);

                // Fix verbosity
                // 打印输出中间的信息，设置为安静模式
                ORB_SLAM3::Verbose::SetTh(ORB_SLAM3::Verbose::VERBOSITY_QUIET);
            }
        };
    }
}