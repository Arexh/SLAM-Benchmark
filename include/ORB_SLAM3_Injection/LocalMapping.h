#pragma once

#include "ORB_SLAM3_detailed_comments/include/LocalMapping.h"
#include "ORB_SLAM3_detailed_comments/include/Optimizer.h"

namespace SLAM_Benchmark
{
    namespace ORB_SLAM3_Inject
    {
        class LocalMapping : public ORB_SLAM3::LocalMapping
        {
        public:
            LocalMapping(ORB_SLAM3::System *pSys, ORB_SLAM3::Atlas *pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName) : ORB_SLAM3::LocalMapping(pSys, pAtlas, bMonocular, bInertial, _strSeqName) {}

            void Run() override
            {

                // 标记状态，表示当前run函数正在运行，尚未结束
                mbFinished = false;
                // 主循环
                while (1)
                {
                    // Tracking will see that Local Mapping is busy
                    // Step 1 告诉Tracking，LocalMapping正处于繁忙状态，请不要给我发送关键帧打扰我
                    // LocalMapping线程处理的关键帧都是Tracking线程发过来的
                    SetAcceptKeyFrames(false);

                    // Check if there are keyframes in the queue
                    // 等待处理的关键帧列表不为空 并且imu正常
                    if (CheckNewKeyFrames() && !mbBadImu)
                    {

#ifdef REGISTER_TIMES
                        double timeLBA_ms = 0;
                        double timeKFCulling_ms = 0;

                        std::chrono::steady_clock::time_point time_StartProcessKF = std::chrono::steady_clock::now();
#endif
                        // BoW conversion and insertion in Map
                        // Step 2 处理列表中的关键帧，包括计算BoW、更新观测、描述子、共视图，插入到地图等
                        ProcessNewKeyFrame();
#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_EndProcessKF = std::chrono::steady_clock::now();

                        double timeProcessKF = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndProcessKF - time_StartProcessKF).count();
                        vdKFInsert_ms.push_back(timeProcessKF);
#endif
                        // Check recent MapPoints
                        // Step 3 根据地图点的观测情况剔除质量不好的地图点
                        MapPointCulling();
#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_EndMPCulling = std::chrono::steady_clock::now();

                        double timeMPCulling = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndMPCulling - time_EndProcessKF).count();
                        vdMPCulling_ms.push_back(timeMPCulling);
#endif
                        // Triangulate new MapPoints
                        // Step 4 当前关键帧与相邻关键帧通过三角化产生新的地图点，使得跟踪更稳
                        CreateNewMapPoints();

                        mbAbortBA = false; // 注意orbslam2中放在了函数SearchInNeighbors（用到了mbAbortBA）后面，应该放这里更合适
                                           // 已经处理完队列中的最后的一个关键帧
                        if (!CheckNewKeyFrames())
                        {
                            // Find more matches in neighbor keyframes and fuse point duplications
                            //  Step 5 检查并融合当前关键帧与相邻关键帧帧（两级相邻）中重复的地图点
                            // 先完成相邻关键帧与当前关键帧的地图点的融合（在相邻关键帧中查找当前关键帧的地图点），
                            // 再完成当前关键帧与相邻关键帧的地图点的融合（在当前关键帧中查找当前相邻关键帧的地图点）
                            SearchInNeighbors();
                        }

#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_EndMPCreation = std::chrono::steady_clock::now();

                        double timeMPCreation = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndMPCreation - time_EndMPCulling).count();
                        vdMPCreation_ms.push_back(timeMPCreation);
#endif

                        bool b_doneLBA = false;
                        int num_FixedKF_BA = 0;
                        int num_OptKF_BA = 0;
                        int num_MPs_BA = 0;
                        int num_edges_BA = 0;
                        // 已经处理完队列中的最后的一个关键帧，并且闭环检测没有请求停止LocalMapping
                        if (!CheckNewKeyFrames() && !stopRequested())
                        {
                            // 当前地图中关键帧数目大于2个
                            if (mpAtlas->KeyFramesInMap() > 2)
                            {
                                // Step 6.1 处于IMU模式并且当前关键帧所在的地图已经完成IMU初始化
                                if (mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized())
                                {
                                    // 计算上一关键帧到当前关键帧相机光心的距离 + 上上关键帧到上一关键帧相机光心的距离
                                    float dist = cv::norm(mpCurrentKeyFrame->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->GetCameraCenter()) +
                                                 cv::norm(mpCurrentKeyFrame->mPrevKF->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->mPrevKF->GetCameraCenter());
                                    // 如果距离大于5厘米，记录当前KF和上一KF时间戳的差，累加到mTinit
                                    if (dist > 0.05)
                                        mTinit += mpCurrentKeyFrame->mTimeStamp - mpCurrentKeyFrame->mPrevKF->mTimeStamp;
                                    // 当前关键帧所在的地图尚未完成IMU BA2（IMU第三阶段初始化）
                                    if (!mpCurrentKeyFrame->GetMap()->GetIniertialBA2())
                                    {
                                        // 如果累计时间差小于10s 并且 距离小于2厘米，认为运动幅度太小，不足以初始化IMU，将mbBadImu设置为true
                                        if ((mTinit < 10.f) && (dist < 0.02))
                                        {
                                            cout << "Not enough motion for initializing. Reseting..." << endl;
                                            unique_lock<mutex> lock(mMutexReset);
                                            mbResetRequestedActiveMap = true;
                                            mpMapToReset = mpCurrentKeyFrame->GetMap();
                                            mbBadImu = true; //在跟踪线程里会重置当前活跃地图
                                        }
                                    }
                                    // 判断成功跟踪匹配的点数是否足够多
                                    // 条件---------1.1、跟踪成功的内点数目大于75-----1.2、并且是单目--或--2.1、跟踪成功的内点数目大于100-----2.2、并且不是单目
                                    bool bLarge = ((mpTracker->GetMatchesInliers() > 75) && mbMonocular) || ((mpTracker->GetMatchesInliers() > 100) && !mbMonocular);
                                    // 局部地图+IMU一起优化，优化关键帧位姿、地图点、IMU参数
                                    ORB_SLAM3::Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(), num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA, bLarge, !mpCurrentKeyFrame->GetMap()->GetIniertialBA2());
                                    b_doneLBA = true;
                                }
                                else
                                {
                                    // Step 6.2 不是IMU模式或者当前关键帧所在的地图还未完成IMU初始化
                                    // 局部地图BA，不包括IMU数据
                                    // 注意这里的第二个参数是按地址传递的,当这里的 mbAbortBA 状态发生变化时，能够及时执行/停止BA
                                    // 局部地图优化，不包括IMU信息。优化关键帧位姿、地图点
                                    ORB_SLAM3::Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(), num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA);
                                    b_doneLBA = true;
                                }
                            }
#ifdef REGISTER_TIMES
                            std::chrono::steady_clock::time_point time_EndLBA = std::chrono::steady_clock::now();

                            if (b_doneLBA)
                            {
                                timeLBA_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndLBA - time_EndMPCreation).count();
                                vdLBASync_ms.push_back(timeLBA_ms);

                                nLBA_exec += 1;
                                if (mbAbortBA)
                                {
                                    nLBA_abort += 1;
                                }
                                vnLBA_edges.push_back(num_edges_BA);
                                vnLBA_KFopt.push_back(num_OptKF_BA);
                                vnLBA_KFfixed.push_back(num_FixedKF_BA);
                                vnLBA_MPs.push_back(num_MPs_BA);
                            }

#endif

                            // Initialize IMU here
                            // Step 7 当前关键帧所在地图未完成IMU初始化（第一阶段）
                            if (!mpCurrentKeyFrame->GetMap()->isImuInitialized() && mbInertial)
                            {
                                // 在函数InitializeIMU里设置IMU成功初始化标志 SetImuInitialized
                                // IMU第一阶段初始化
                                if (mbMonocular)
                                    InitializeIMU(1e2, 1e10, true);
                                else
                                    InitializeIMU(1e2, 1e5, true);
                            }

                            // Check redundant local Keyframes
                            // 跟踪中关键帧插入条件比较松，交给LocalMapping线程的关键帧会比较密，这里再删除冗余
                            // Step 8 检测并剔除当前帧相邻的关键帧中冗余的关键帧
                            // 冗余的判定：该关键帧的90%的地图点可以被其它关键帧观测到
                            KeyFrameCulling();

#ifdef REGISTER_TIMES
                            std::chrono::steady_clock::time_point time_EndKFCulling = std::chrono::steady_clock::now();

                            timeKFCulling_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndKFCulling - time_EndLBA).count();
                            vdKFCullingSync_ms.push_back(timeKFCulling_ms);
#endif
                            // Step 9 如果距离IMU第一阶段初始化成功累计时间差小于100s，进行VIBA
                            if ((mTinit < 100.0f) && mbInertial)
                            {
                                // Step 9.1 根据条件判断是否进行VIBA1（IMU第二阶段初始化）
                                // 条件：1、当前关键帧所在的地图还未完成IMU初始化---并且--------2、正常跟踪状态----------
                                if (mpCurrentKeyFrame->GetMap()->isImuInitialized() && mpTracker->mState == ORB_SLAM3::Tracking::OK) // Enter here everytime local-mapping is called
                                {
                                    // 当前关键帧所在的地图还未完成VIBA 1
                                    if (!mpCurrentKeyFrame->GetMap()->GetIniertialBA1())
                                    {
                                        // 如果累计时间差大于5s，开始VIBA1（IMU第二阶段初始化）
                                        if (mTinit > 5.0f)
                                        {
                                            cout << "start VIBA 1" << endl;
                                            mpCurrentKeyFrame->GetMap()->SetIniertialBA1();
                                            if (mbMonocular)
                                                InitializeIMU(1.f, 1e5, true); // 1.f, 1e5
                                            else
                                                InitializeIMU(1.f, 1e5, true); // 1.f, 1e5

                                            cout << "end VIBA 1" << endl;
                                        }
                                    }
                                    //else if (mbNotBA2){
                                    // Step 9.2 根据条件判断是否进行VIBA2（IMU第三阶段初始化）
                                    // 当前关键帧所在的地图还未完成VIBA 2
                                    else if (!mpCurrentKeyFrame->GetMap()->GetIniertialBA2())
                                    {
                                        // 如果累计时间差大于15s，开始VIBA2（IMU第三阶段初始化）
                                        if (mTinit > 15.0f)
                                        { // 15.0f
                                            cout << "start VIBA 2" << endl;
                                            mpCurrentKeyFrame->GetMap()->SetIniertialBA2();
                                            if (mbMonocular)
                                                InitializeIMU(0.f, 0.f, true); // 0.f, 0.f
                                            else
                                                InitializeIMU(0.f, 0.f, true);

                                            cout << "end VIBA 2" << endl;
                                        }
                                    }

                                    // scale refinement
                                    // Step 9.3 在关键帧小于100时，会在满足一定时间间隔后多次进行尺度、重力方向优化
                                    if (((mpAtlas->KeyFramesInMap()) <= 100) &&
                                        ((mTinit > 25.0f && mTinit < 25.5f) ||
                                         (mTinit > 35.0f && mTinit < 35.5f) ||
                                         (mTinit > 45.0f && mTinit < 45.5f) ||
                                         (mTinit > 55.0f && mTinit < 55.5f) ||
                                         (mTinit > 65.0f && mTinit < 65.5f) ||
                                         (mTinit > 75.0f && mTinit < 75.5f)))
                                    {
                                        cout << "start scale ref" << endl;
                                        if (mbMonocular)
                                            // 使用了所有关键帧，但只优化尺度和重力方向
                                            ScaleRefinement();
                                        cout << "end scale ref" << endl;
                                    }
                                }
                            }
                        }

#ifdef REGISTER_TIMES
                        vdLBA_ms.push_back(timeLBA_ms);
                        vdKFCulling_ms.push_back(timeKFCulling_ms);
#endif

                        // Step 10 将当前帧加入到闭环检测队列中
                        mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);

#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_EndLocalMap = std::chrono::steady_clock::now();

                        double timeLocalMap = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndLocalMap - time_StartProcessKF).count();
                        vdLMTotal_ms.push_back(timeLocalMap);
#endif
                    }
                    else if (Stop() && !mbBadImu) // 当要终止当前线程的时候
                    {
                        // Safe area to stop
                        while (isStopped() && !CheckFinish())
                        {
                            // cout << "LM: usleep if is stopped" << endl;
                            // 如果还没有结束利索,那么等等它
                            usleep(3000);
                        }
                        // 然后确定终止了就跳出这个线程的主循环
                        if (CheckFinish())
                            break;
                    }

                    // 查看是否有复位线程的请求
                    ResetIfRequested();

                    // Tracking will see that Local Mapping is busy
                    SetAcceptKeyFrames(true);

                    // 如果当前线程已经结束了就跳出主循环
                    if (CheckFinish())
                        break;

                    // cout << "LM: normal usleep" << endl;
                    usleep(3000);
                }

                // 设置线程已经终止
                SetFinish();
            }
        };
    }
}
