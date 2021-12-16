#pragma once

#include "ORB_SLAM2_detailed_comments/include/System.h"
#include <unistd.h>
#include <iostream>
#include <future>
#include <thread>

#include "ThreadRecorder.h"
#include "SystemRecorder.h"
#include "ORB_SLAM2_Injection/LocalMapping.h"
#include "ORB_SLAM2_Injection/LoopClosing.h"

namespace SLAM_Benchmark
{
    namespace ORB_SLAM2_Inject
    {
        class System : public ORB_SLAM2::System
        {
        public:
            System(const string &strVocFile,
                   const string &strSettingsFile,
                   const eSensor sensor,
                   const bool bUseViewer = true)
            {
                /* init members */
                mSensor = sensor;
                mpViewer = static_cast<ORB_SLAM2::Viewer *>(NULL);
                mbReset = false;
                mbActivateLocalizationMode = false;
                mbDeactivateLocalizationMode = false;
                /* init members */

                /* init recorders */
                SLAM_Benchmark::SystemRecorder *system_recorder = SLAM_Benchmark::SystemRecorder::getInstance(SLAM_Benchmark::SystemName::ORB_SLAM2);
                SLAM_Benchmark::ThreadRecorder *local_mapper_recorder = new SLAM_Benchmark::ThreadRecorder("LocalMapping");
                SLAM_Benchmark::ThreadRecorder *loop_closing_recorder = new SLAM_Benchmark::ThreadRecorder("LoopClosing");
                SLAM_Benchmark::ThreadRecorder *bundle_adjustment_recorder = new SLAM_Benchmark::ThreadRecorder("BundleAdjustment");
                system_recorder->addThreadRecord(local_mapper_recorder);
                system_recorder->addThreadRecord(loop_closing_recorder);
                system_recorder->addThreadRecord(bundle_adjustment_recorder);
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

                // Output welcome message
                cout << endl
                     << "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl
                     << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
                     << "This is free software, and you are welcome to redistribute it" << endl
                     << "under certain conditions. See LICENSE.txt." << endl
                     << endl;

                // 输出当前传感器类型
                cout << "Input sensor was set to: ";

                if (mSensor == MONOCULAR)
                    cout << "Monocular" << endl;
                else if (mSensor == STEREO)
                    cout << "Stereo" << endl;
                else if (mSensor == RGBD)
                    cout << "RGB-D" << endl;

                //Check settings file
                cv::FileStorage fsSettings(strSettingsFile.c_str(), //将配置文件名转换成为字符串
                                           cv::FileStorage::READ);  //只读
                //如果打开失败，就输出调试信息
                if (!fsSettings.isOpened())
                {
                    cerr << "Failed to open settings file at: " << strSettingsFile << endl;
                    //然后退出
                    exit(-1);
                }

                //Load ORB Vocabulary
                cout << endl
                     << "Loading ORB Vocabulary. This could take a while..." << endl;

                //建立一个新的ORB字典
                mpVocabulary = new ORB_SLAM2::ORBVocabulary();
                //获取字典加载状态
                bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
                //如果加载失败，就输出调试信息
                if (!bVocLoad)
                {
                    cerr << "Wrong path to vocabulary. " << endl;
                    cerr << "Falied to open at: " << strVocFile << endl;
                    //然后退出
                    exit(-1);
                }
                //否则则说明加载成功
                cout << "Vocabulary loaded!" << endl
                     << endl;

                //Create KeyFrame Database
                mpKeyFrameDatabase = new ORB_SLAM2::KeyFrameDatabase(*mpVocabulary);

                //Create the Map
                mpMap = new ORB_SLAM2::Map();

                //Create Drawers. These are used by the Viewer
                //这里的帧绘制器和地图绘制器将会被可视化的Viewer所使用
                mpFrameDrawer = new ORB_SLAM2::FrameDrawer(mpMap);
                mpMapDrawer = new ORB_SLAM2::MapDrawer(mpMap, strSettingsFile);

                //在本主进程中初始化追踪线程
                //Initialize the Tracking thread
                //(it will live in the main thread of execution, the one that called this constructor)
                mpTracker = new ORB_SLAM2::Tracking(this,               //现在还不是很明白为什么这里还需要一个this指针  TODO
                                                    mpVocabulary,       //字典
                                                    mpFrameDrawer,      //帧绘制器
                                                    mpMapDrawer,        //地图绘制器
                                                    mpMap,              //地图
                                                    mpKeyFrameDatabase, //关键帧地图
                                                    strSettingsFile,    //设置文件路径
                                                    mSensor);           //传感器类型iomanip

                LocalMapping *mpLocalMapperInjected = new LocalMapping(mpMap,                 //指定使iomanip
                                                                       mSensor == MONOCULAR); // TODO 为什么这个要设置成为MONOCULAR？？？

                //初始化局部建图线程并运行
                //Initialize the Local Mapping thread and launch
                mpLocalMapper = mpLocalMapperInjected;

                //运行这个局部建图线程
                mptLocalMapping = new thread(&LocalMapping::Run,                            //这个线程会调用的函数
                                             mpLocalMapperInjected); //这个调用函数的参数

                LoopClosing *mpLoopCloserInjected = new LoopClosing(mpMap,                 //地图
                                                                    mpKeyFrameDatabase,    //关键帧数据库
                                                                    mpVocabulary,          //ORB字典
                                                                    mSensor != MONOCULAR); //当前的传感器是否是单目

                //Initialize the Loop Closing thread and launchiomanip
                mpLoopCloser = mpLoopCloserInjected;
                //创建回环检测线程
                mptLoopClosing = new thread(&LoopClosing::Run,                            //线程的主函数
                                            mpLoopCloserInjected); //该函数的参数

                //Initialize the Viewer thread and launch
                if (bUseViewer)
                {
                    //如果指定了，程序的运行过程中需要运行可视化部分
                    //新建viewer
                    mpViewer = new ORB_SLAM2::Viewer(this,             //又是这个
                                                     mpFrameDrawer,    //帧绘制器
                                                     mpMapDrawer,      //地图绘制器
                                                     mpTracker,        //追踪器
                                                     strSettingsFile); //配置文件的访问路径
                    //新建viewer线程
                    mptViewer = new thread(&ORB_SLAM2::Viewer::Run, mpViewer);
                    //给运动追踪器设置其查看器
                    mpTracker->SetViewer(mpViewer);
                }

                //Set pointers between threads
                //设置进程间的指针
                mpTracker->SetLocalMapper(mpLocalMapper);
                mpTracker->SetLoopClosing(mpLoopCloser);

                mpLocalMapper->SetTracker(mpTracker);
                mpLocalMapper->SetLoopCloser(mpLoopCloser);

                mpLoopCloser->SetTracker(mpTracker);
                mpLoopCloser->SetLocalMapper(mpLocalMapper);
            }
        };
    } // ORB_SLAM2
} // SLAM_Benchmark