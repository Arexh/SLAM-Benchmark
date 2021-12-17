#pragma once

#include "ORB_SLAM3_detailed_comments/include/Tracking.h"

#include "ThreadRecorder.h"
#include "SystemRecorder.h"

namespace SLAM_Benchmark
{
    namespace ORB_SLAM3_Inject
    {
        class Tracking : public ORB_SLAM3::Tracking
        {
        public:
            Tracking(ORB_SLAM3::System *pSys, ORB_SLAM3::ORBVocabulary *pVoc, ORB_SLAM3::FrameDrawer *pFrameDrawer, ORB_SLAM3::MapDrawer *pMapDrawer, ORB_SLAM3::Atlas *pAtlas,
                     ORB_SLAM3::KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor, const string &_nameSeq = std::string()) : ORB_SLAM3::Tracking(pSys, pVoc, pFrameDrawer, pMapDrawer, pAtlas, pKFDB, strSettingPath, sensor, _nameSeq) {}

            /**
             * @brief 
             * 输入左目RGB或RGBA图像，输出世界坐标系到该帧相机坐标系的变换矩阵
             * 
             * @param[in] im 单目图像
             * @param[in] timestamp 时间戳
             * @return cv::Mat 
             * 
             * Step 1 ：将彩色图像转为灰度图像
             * Step 2 ：构造Frame
             * Step 3 ：跟踪
            */
            cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename) override
            {
                SLAM_Benchmark::ThreadRecorder *thread_recorder = SLAM_Benchmark::SystemRecorder::getInstance(SLAM_Benchmark::SystemName::ORB_SLAM3)->getThreadRecorder("Tracking");

                mImGray = im;

                // Step 1 ：将彩色图像转为灰度图像
                //若图片是3、4通道的彩色图，还需要转化成单通道灰度图
                if (mImGray.channels() == 3)
                {
                    if (mbRGB)
                        cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
                    else
                        cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
                }
                else if (mImGray.channels() == 4)
                {
                    if (mbRGB)
                        cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
                    else
                        cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
                }

                thread_recorder->recordSubprocessStart("ORBExtraction");
                // Step 2 ：构造Frame类
                if (mSensor == ORB_SLAM3::System::MONOCULAR)
                {
                    if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET || (lastID - initID) < mMaxFrames)
                        mCurrentFrame = ORB_SLAM3::Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth);
                    else
                        mCurrentFrame = ORB_SLAM3::Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth);
                }
                else if (mSensor == ORB_SLAM3::System::IMU_MONOCULAR)
                {
                    //判断该帧是不是初始化
                    if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET) //没有成功初始化的前一个状态就是NO_IMAGES_YET
                    {
                        mCurrentFrame = ORB_SLAM3::Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth, &mLastFrame, *mpImuCalib);
                    }
                    else
                        mCurrentFrame = ORB_SLAM3::Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth, &mLastFrame, *mpImuCalib);
                }
                thread_recorder->recordSubprocessStop("ORBExtraction");

                // t0存储未初始化时的第1帧图像时间戳
                if (mState == NO_IMAGES_YET)
                    t0 = timestamp;

                mCurrentFrame.mNameFile = filename;
                mCurrentFrame.mnDataset = mnNumDataset;

                lastID = mCurrentFrame.mnId;

                // Step 3 ：跟踪
                Track();

                //返回当前帧的位姿
                return mCurrentFrame.mTcw.clone();
            }

            /**
             * @brief 跟踪过程，包括恒速模型跟踪、参考关键帧跟踪、局部地图跟踪
             * track包含两部分：估计运动、跟踪局部地图
             * 
             * Step 1：初始化
             * Step 2：跟踪
             * Step 3：记录位姿信息，用于轨迹复现
             */
            void Track() override
            {
                SLAM_Benchmark::ThreadRecorder *thread_recorder = SLAM_Benchmark::SystemRecorder::getInstance(SLAM_Benchmark::SystemName::ORB_SLAM3)->getThreadRecorder("Tracking");

                if (bStepByStep)
                {
                    while (!mbStep)
                        usleep(500);
                    mbStep = false;
                }

                thread_recorder->recordSubprocessStart("Track");

                // Step 1 如局部建图里认为IMU有问题，重置当前活跃地图
                if (mpLocalMapper->mbBadImu)
                {
                    cout << "TRACK: Reset map because local mapper set the bad imu flag " << endl;
                    mpSystem->ResetActiveMap();
                    return;
                }

                // 从Atlas中取出当前active的地图
                ORB_SLAM3::Map *pCurrentMap = mpAtlas->GetCurrentMap();

                // Step 2 处理时间戳异常的情况
                if (mState != NO_IMAGES_YET)
                {
                    // 进入以下两个if语句都是不正常的情况，不进行跟踪直接返回
                    if (mLastFrame.mTimeStamp > mCurrentFrame.mTimeStamp)
                    {
                        // 如果当前图像时间戳比前一帧图像时间戳小，说明出错了，清除imu数据，创建新的子地图
                        cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
                        unique_lock<mutex> lock(mMutexImuQueue);
                        mlQueueImuData.clear();
                        // 创建新地图
                        CreateMapInAtlas();
                        return;
                    }
                    else if (mCurrentFrame.mTimeStamp > mLastFrame.mTimeStamp + 1.0)
                    {
                        // 如果当前图像时间戳和前一帧图像时间戳大于1s，说明时间戳明显跳变了，重置地图后直接返回
                        cout << "id last: " << mLastFrame.mnId << "    id curr: " << mCurrentFrame.mnId << endl;
                        //根据是否是imu模式,进行imu的补偿
                        if (mpAtlas->isInertial())
                        {
                            // 如果当前地图imu成功初始化
                            if (mpAtlas->isImuInitialized())
                            {
                                cout << "Timestamp jump detected. State set to LOST. Reseting IMU integration..." << endl;
                                // IMU完成第2阶段BA（在localmapping线程里）
                                if (!pCurrentMap->GetIniertialBA2())
                                {
                                    // 如果当前子图中imu没有经过BA2，重置active地图
                                    mpSystem->ResetActiveMap();
                                }
                                else
                                {
                                    // 如果当前子图中imu进行了BA2，重新创建新的子图
                                    CreateMapInAtlas();
                                }
                            }
                            else
                            {
                                // 如果当前子图中imu还没有初始化，重置active地图
                                cout << "Timestamp jump detected, before IMU initialization. Reseting..." << endl;
                                mpSystem->ResetActiveMap();
                            }
                        }
                        // 不跟踪直接返回
                        return;
                    }
                }

                // Step 3 IMU模式下设置IMU的Bias参数,还要保证上一帧存在
                if ((mSensor == ORB_SLAM3::System::IMU_MONOCULAR || mSensor == ORB_SLAM3::System::IMU_STEREO) && mpLastKeyFrame)
                    //认为bias在两帧间不变
                    mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

                if (mState == NO_IMAGES_YET)
                {
                    mState = NOT_INITIALIZED;
                }

                mLastProcessedState = mState;
                // Step 4 IMU模式且没有创建地图的情况下对IMU数据进行预积分
                if ((mSensor == ORB_SLAM3::System::IMU_MONOCULAR || mSensor == ORB_SLAM3::System::IMU_STEREO) && !mbCreatedMap)
                {
                    // IMU数据进行预积分
                    PreintegrateIMU();
                }
                mbCreatedMap = false;

                // Get Map Mutex -> Map cannot be changed
                // 地图更新时加锁。保证地图不会发生变化
                // 疑问:这样子会不会影响地图的实时更新?
                // 回答：主要耗时在构造帧中特征点的提取和匹配部分,在那个时候地图是没有被上锁的,有足够的时间更新地图
                unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

                mbMapUpdated = false;

                // 判断地图id是否更新了
                int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
                int nMapChangeIndex = pCurrentMap->GetLastMapChange();
                if (nCurMapChangeIndex > nMapChangeIndex)
                {
                    // cout << "Map update detected" << endl;
                    // 检测到地图更新了
                    pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
                    mbMapUpdated = true;
                }

                if (mState == NOT_INITIALIZED)
                {
                    // Step 5 初始化
                    if (mSensor == ORB_SLAM3::System::STEREO || mSensor == ORB_SLAM3::System::RGBD || mSensor == ORB_SLAM3::System::IMU_STEREO)
                    {
                        //双目RGBD相机的初始化共用一个函数
                        StereoInitialization();
                    }
                    else
                    {
                        //单目初始化
                        MonocularInitialization();
                    }

                    mpFrameDrawer->Update(this);

                    if (mState != OK) // If rightly initialized, mState=OK
                    {
                        // 如果没有成功初始化，直接返回
                        mLastFrame = ORB_SLAM3::Frame(mCurrentFrame);
                        return;
                    }

                    if (mpAtlas->GetAllMaps().size() == 1)
                    {
                        // 如果当前地图是第一个地图，记录当前帧id为第一帧
                        mnFirstFrameId = mCurrentFrame.mnId;
                    }
                }
                else
                {
                    // System is initialized. Track Frame.
                    // Step 6 系统成功初始化，下面是具体跟踪过程
                    bool bOK;

                    // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
                    // mbOnlyTracking等于false表示正常SLAM模式（定位+地图更新），mbOnlyTracking等于true表示仅定位模式
                    // tracking 类构造时默认为false。在viewer中有个开关ActivateLocalizationMode，可以控制是否开启mbOnlyTracking
                    if (!mbOnlyTracking)
                    {

                        // State OK
                        // Local Mapping is activated. This is the normal behaviour, unless
                        // you explicitly activate the "only tracking" mode.
                        // 跟踪进入正常SLAM模式，有地图更新

                        // 如果正常跟踪
                        if (mState == OK)
                        {

                            // Local Mapping might have changed some MapPoints tracked in last frame
                            // Step 6.1 检查并更新上一帧被替换的MapPoints
                            // 局部建图线程则可能会对原有的地图点进行替换.在这里进行检查
                            CheckReplacedInLastFrame();

                            // Step 6.2 运动模型是空的并且imu未初始化或刚完成重定位，跟踪参考关键帧；否则恒速模型跟踪
                            // 第一个条件,如果运动模型为空并且imu未初始化,说明是刚开始第一帧跟踪，或者已经跟丢了。
                            // 第二个条件,如果当前帧紧紧地跟着在重定位的帧的后面，我们用重定位帧来恢复位姿
                            // mnLastRelocFrameId 上一次重定位的那一帧
                            if ((mVelocity.empty() && !pCurrentMap->isImuInitialized()) || mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                            {
                                //Verbose::PrintMess("TRACK: Track with respect to the reference KF ", Verbose::VERBOSITY_DEBUG);
                                bOK = TrackReferenceKeyFrame();
                            }
                            else
                            {
                                //Verbose::PrintMess("TRACK: Track with motion model", Verbose::VERBOSITY_DEBUG);
                                // 用恒速模型跟踪。所谓的恒速就是假设上上帧到上一帧的位姿=上一帧的位姿到当前帧位姿
                                // 根据恒速模型设定当前帧的初始位姿，用最近的普通帧来跟踪当前的普通帧
                                // 通过投影的方式在参考帧中找当前帧特征点的匹配点，优化每个特征点所对应3D点的投影误差即可得到位姿
                                bOK = TrackWithMotionModel();
                                if (!bOK)
                                    //根据恒速模型失败了，只能根据参考关键帧来跟踪
                                    bOK = TrackReferenceKeyFrame();
                            }

                            // 新增了一个状态RECENTLY_LOST，主要是结合IMU看看能不能拽回来
                            // Step 6.3 如果经过跟踪参考关键帧、恒速模型跟踪都失败的话，并满足一定条件就要标记为RECENTLY_LOST或LOST
                            if (!bOK)
                            {
                                // 条件1：如果当前帧距离上次重定位成功不到1s
                                //        mnFramesToResetIMU 表示经过多少帧后可以重置IMU，一般设置为和帧率相同，对应的时间是1s
                                // 条件2：单目+IMU 或者 双目+IMU模式
                                // 同时满足条件1，2，标记为LOST
                                if (mCurrentFrame.mnId <= (mnLastRelocFrameId + mnFramesToResetIMU) &&
                                    (mSensor == ORB_SLAM3::System::IMU_MONOCULAR || mSensor == ORB_SLAM3::System::IMU_STEREO))
                                {
                                    mState = LOST;
                                }
                                else if (pCurrentMap->KeyFramesInMap() > 10)
                                {
                                    // 条件1：当前地图中关键帧数目较多（大于10）
                                    // 条件2（隐藏条件）：当前帧距离上次重定位帧超过1s（说明还比较争气，值的救）或者非IMU模式
                                    // 同时满足条件1，2，则将状态标记为RECENTLY_LOST，后面会结合IMU预测的位姿看看能不能拽回来
                                    cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
                                    mState = RECENTLY_LOST;
                                    // 记录丢失时间
                                    mTimeStampLost = mCurrentFrame.mTimeStamp;
                                    //mCurrentFrame.SetPose(mLastFrame.mTcw);
                                }
                                else
                                {
                                    mState = LOST;
                                }
                            }
                        }
                        else //跟踪不正常按照下面处理
                        {
                            if (mState == RECENTLY_LOST)
                            {
                                // 如果是RECENTLY_LOST状态
                                ORB_SLAM3::Verbose::PrintMess("Lost for a short time", ORB_SLAM3::Verbose::VERBOSITY_NORMAL);
                                // bOK先置为true
                                bOK = true;

                                if ((mSensor == ORB_SLAM3::System::IMU_MONOCULAR || mSensor == ORB_SLAM3::System::IMU_STEREO))
                                {
                                    // IMU模式下可以用IMU来预测位姿，看能否拽回来
                                    // Step 6.4 如果当前地图中IMU已经成功初始化，就用IMU数据预测位姿
                                    if (pCurrentMap->isImuInitialized())
                                        PredictStateIMU();
                                    else
                                        bOK = false;

                                    // 如果IMU模式下当前帧距离跟丢帧超过5s还没有找回（time_recently_lost默认为5s）
                                    // 放弃了，将RECENTLY_LOST状态改为LOST
                                    if (mCurrentFrame.mTimeStamp - mTimeStampLost > time_recently_lost)
                                    {
                                        mState = LOST;
                                        ORB_SLAM3::Verbose::PrintMess("Track Lost...", ORB_SLAM3::Verbose::VERBOSITY_NORMAL);
                                        bOK = false;
                                    }
                                }
                                else
                                {
                                    // Step 6.5 纯视觉模式则进行重定位。主要是BOW搜索，EPnP求解位姿
                                    // TODO fix relocalization
                                    bOK = Relocalization();
                                    if (!bOK)
                                    {
                                        // 纯视觉模式下重定位失败，状态为LOST
                                        mState = LOST;
                                        ORB_SLAM3::Verbose::PrintMess("Track Lost...", ORB_SLAM3::Verbose::VERBOSITY_NORMAL);
                                        bOK = false;
                                    }
                                }
                            }
                            else if (mState == LOST) // 上一帧为最近丢失且重定位失败时
                            {
                                // Step 6.6 如果是LOST状态
                                // 开启一个新地图
                                ORB_SLAM3::Verbose::PrintMess("A new map is started...", ORB_SLAM3::Verbose::VERBOSITY_NORMAL);

                                if (pCurrentMap->KeyFramesInMap() < 10)
                                {
                                    // 当前地图中关键帧数目小于10，重置当前地图
                                    mpSystem->ResetActiveMap();
                                    cout << "Reseting current map..." << endl;
                                }
                                else
                                    // 当前地图中关键帧数目超过10，创建新地图
                                    CreateMapInAtlas();
                                // 干掉上一个关键帧
                                if (mpLastKeyFrame)
                                    mpLastKeyFrame = static_cast<ORB_SLAM3::KeyFrame *>(NULL);

                                ORB_SLAM3::Verbose::PrintMess("done", ORB_SLAM3::Verbose::VERBOSITY_NORMAL);

                                return;
                            }
                        }
                    }
                    else // 纯定位模式
                    {
                        // Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)
                        // 只进行跟踪tracking，局部地图不工作
                        if (mState == LOST)
                        {
                            if (mSensor == ORB_SLAM3::System::IMU_MONOCULAR || mSensor == ORB_SLAM3::System::IMU_STEREO)
                                ORB_SLAM3::Verbose::PrintMess("IMU. State LOST", ORB_SLAM3::Verbose::VERBOSITY_NORMAL);
                            // Step 6.1 LOST状态进行重定位
                            bOK = Relocalization();
                        }
                        else
                        {
                            // mbVO是mbOnlyTracking为true时的才有的一个变量
                            // mbVO为false表示此帧匹配了很多的MapPoints，跟踪很正常 (注意有点反直觉)
                            // mbVO为true表明此帧匹配了很少的MapPoints，少于10个，要跟丢
                            if (!mbVO)
                            {
                                // In last frame we tracked enough MapPoints in the map
                                // Step 6.2 如果跟踪状态正常，使用恒速模型或参考关键帧跟踪
                                if (!mVelocity.empty())
                                {
                                    // 优先使用恒速模型跟踪
                                    bOK = TrackWithMotionModel();
                                }
                                else
                                {
                                    // 如果恒速模型不被满足,那么就只能够通过参考关键帧来跟踪
                                    bOK = TrackReferenceKeyFrame();
                                }
                            }
                            else
                            {
                                // In last frame we tracked mainly "visual odometry" points.

                                // We compute two camera poses, one from motion model and one doing relocalization.
                                // If relocalization is sucessfull we choose that solution, otherwise we retain
                                // the "visual odometry" solution.

                                // mbVO为true，表明此帧匹配了很少（小于10）的地图点，要跟丢的节奏，既做跟踪又做重定位

                                // MM=Motion Model,通过运动模型进行跟踪的结果
                                bool bOKMM = false;
                                // 通过重定位方法来跟踪的结果
                                bool bOKReloc = false;
                                // 运动模型中构造的地图点
                                vector<ORB_SLAM3::MapPoint *> vpMPsMM;
                                // 在追踪运动模型后发现的外点
                                vector<bool> vbOutMM;
                                // 运动模型得到的位姿
                                cv::Mat TcwMM;
                                // Step 6.3 当运动模型有效的时候,根据运动模型计算位姿
                                if (!mVelocity.empty())
                                {
                                    bOKMM = TrackWithMotionModel();
                                    // 将恒速模型跟踪结果暂存到这几个变量中，因为后面重定位会改变这些变量
                                    vpMPsMM = mCurrentFrame.mvpMapPoints;
                                    vbOutMM = mCurrentFrame.mvbOutlier;
                                    TcwMM = mCurrentFrame.mTcw.clone();
                                }
                                // Step 6.4 使用重定位的方法来得到当前帧的位姿
                                bOKReloc = Relocalization();

                                // Step 6.5 根据前面的恒速模型、重定位结果来更新状态
                                if (bOKMM && !bOKReloc)
                                {
                                    // 恒速模型成功、重定位失败，重新使用之前暂存的恒速模型结果
                                    mCurrentFrame.SetPose(TcwMM);
                                    mCurrentFrame.mvpMapPoints = vpMPsMM;
                                    mCurrentFrame.mvbOutlier = vbOutMM;

                                    //? 疑似bug！这段代码是不是重复增加了观测次数？后面 TrackLocalMap 函数中会有这些操作
                                    // 如果当前帧匹配的3D点很少，增加当前可视地图点的被观测次数
                                    if (mbVO)
                                    {
                                        // 更新当前帧的地图点被观测次数
                                        for (int i = 0; i < mCurrentFrame.N; i++)
                                        {
                                            // 如果这个特征点形成了地图点,并且也不是外点的时候
                                            if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                            {
                                                // 增加能观测到该地图点的帧数
                                                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                            }
                                        }
                                    }
                                }
                                else if (bOKReloc)
                                {
                                    // 只要重定位成功整个跟踪过程正常进行（重定位与跟踪，更相信重定位）
                                    mbVO = false;
                                }
                                // 有一个成功我们就认为执行成功了
                                bOK = bOKReloc || bOKMM;
                            }
                        }
                    }
                    // 将最新的关键帧作为当前帧的参考关键帧
                    // mpReferenceKF先是上一时刻的参考关键帧，如果当前为新关键帧则变成当前关键帧，如果不是新的关键帧则先为上一帧的参考关键帧，而后经过更新局部关键帧重新确定
                    if (!mCurrentFrame.mpReferenceKF)
                        mCurrentFrame.mpReferenceKF = mpReferenceKF;

                    // Step 7 在跟踪得到当前帧初始姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
                    // 前面只是跟踪一帧得到初始位姿，这里搜索局部关键帧、局部地图点，和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
                    // 在帧间匹配得到初始的姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
                    // local map:当前帧、当前帧的MapPoints、当前关键帧与其它关键帧共视关系
                    // 前面主要是两两跟踪（恒速模型跟踪上一帧、跟踪参考帧），这里搜索局部关键帧后搜集所有局部MapPoints，
                    // 然后将局部MapPoints和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
                    // If we have an initial estimation of the camera pose and matching. Track the local map.
                    if (!mbOnlyTracking)
                    {
                        if (bOK)
                        {
                            // 局部地图跟踪
                            bOK = TrackLocalMap();
                        }
                        if (!bOK)
                            cout << "Fail to track local map!" << endl;
                    }
                    else
                    {
                        // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
                        // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
                        // the camera we will use the local map again.
                        if (bOK && !mbVO)
                            bOK = TrackLocalMap();
                    }
                    // 到此为止跟踪确定位姿阶段结束，下面开始做收尾工作和为下一帧做准备

                    // 查看到此为止时的两个状态变化
                    // bOK的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---true                     -->OK   1 跟踪局部地图成功
                    //          \               \              \---局部地图跟踪失败---false
                    //           \               \---当前帧跟踪失败---false
                    //            \---上一帧跟踪失败---重定位成功---局部地图跟踪成功---true                       -->OK  2 重定位
                    //                          \           \---局部地图跟踪失败---false
                    //                           \---重定位失败---false

                    //
                    // mState的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---OK                  -->OK  1 跟踪局部地图成功
                    //            \               \              \---局部地图跟踪失败---OK                  -->OK  3 正常跟踪
                    //             \               \---当前帧跟踪失败---非OK
                    //              \---上一帧跟踪失败---重定位成功---局部地图跟踪成功---非OK
                    //                            \           \---局部地图跟踪失败---非OK
                    //                             \---重定位失败---非OK（传不到这里，因为直接return了）
                    // 由上图可知当前帧的状态OK的条件是跟踪局部地图成功，重定位或正常跟踪都可
                    // Step 8 根据上面的操作来判断是否追踪成功
                    if (bOK)
                        // 此时还OK才说明跟踪成功了
                        mState = OK;
                    else if (mState == OK) // 由上图可知只有当第一阶段跟踪成功，但第二阶段局部地图跟踪失败时执行
                    {
                        // 带有IMU时状态变为最近丢失，否则直接丢失
                        if (mSensor == ORB_SLAM3::System::IMU_MONOCULAR || mSensor == ORB_SLAM3::System::IMU_STEREO)
                        {
                            ORB_SLAM3::Verbose::PrintMess("Track lost for less than one second...", ORB_SLAM3::Verbose::VERBOSITY_NORMAL);
                            if (!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2())
                            {
                                // IMU模式下IMU没有成功初始化或者没有完成IMU BA，则重置当前地图
                                cout << "IMU is not or recently initialized. Reseting active map..." << endl;
                                mpSystem->ResetActiveMap();
                            }

                            mState = RECENTLY_LOST;
                        }
                        else
                            mState = LOST; // visual to lost

                        // 如果当前帧距离上次重定位帧超过1s，用当前帧时间戳更新lost帧时间戳
                        if (mCurrentFrame.mnId > mnLastRelocFrameId + mMaxFrames)
                        {
                            mTimeStampLost = mCurrentFrame.mTimeStamp;
                        }
                    }

                    // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
                    // 如果刚刚发生重定位并且IMU已经初始化，则保存当前帧信息，重置IMU
                    if ((mCurrentFrame.mnId < (mnLastRelocFrameId + mnFramesToResetIMU)) &&                              // 当前帧距离上次重定位帧小于1s
                        (mCurrentFrame.mnId > mnFramesToResetIMU) &&                                                     // 当前帧已经运行了超过1s
                        ((mSensor == ORB_SLAM3::System::IMU_MONOCULAR) || (mSensor == ORB_SLAM3::System::IMU_STEREO)) && // IMU模式
                        pCurrentMap->isImuInitialized())                                                                 // IMU已经成功初始化
                    {
                        // 存储指针
                        ORB_SLAM3::Verbose::PrintMess("Saving pointer to frame. imu needs reset...", ORB_SLAM3::Verbose::VERBOSITY_NORMAL);
                        ORB_SLAM3::Frame *pF = new ORB_SLAM3::Frame(mCurrentFrame);
                        pF->mpPrevFrame = new ORB_SLAM3::Frame(mLastFrame);

                        // Load preintegration
                        // IMU重置
                        pF->mpImuPreintegratedFrame = new ORB_SLAM3::IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
                    }
                    // 下面代码看起来没有用
                    if (pCurrentMap->isImuInitialized())
                    {
                        if (bOK) // 跟踪成功
                        {
                            // 当前帧距离上次重定位帧刚好等于1s，重置（还未实现 TODO）
                            if (mCurrentFrame.mnId == (mnLastRelocFrameId + mnFramesToResetIMU))
                            {
                                cout << "RESETING FRAME!!!" << endl;
                            }
                            else if (mCurrentFrame.mnId > (mnLastRelocFrameId + 30))
                                mLastBias = mCurrentFrame.mImuBias; // 没啥用，后面会重新赋值后传给普通帧
                        }
                    }

                    // Update drawer
                    // 更新显示线程中的图像、特征点、地图点等信息
                    mpFrameDrawer->Update(this);
                    if (!mCurrentFrame.mTcw.empty())
                        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
                    // 查看到此为止时的两个状态变化
                    // bOK的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---true
                    //          \               \              \---局部地图跟踪失败---false
                    //           \               \---当前帧跟踪失败---false
                    //            \---上一帧跟踪失败---重定位成功---局部地图跟踪成功---true
                    //                          \           \---局部地图跟踪失败---false
                    //                           \---重定位失败---false

                    // mState的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---OK
                    //            \               \              \---局部地图跟踪失败---非OK（IMU时为RECENTLY_LOST）
                    //             \               \---当前帧跟踪失败---非OK(地图超过10个关键帧时 RECENTLY_LOST)
                    //              \---上一帧跟踪失败(RECENTLY_LOST)---重定位成功---局部地图跟踪成功---OK
                    //               \                           \           \---局部地图跟踪失败---LOST
                    //                \                           \---重定位失败---LOST（传不到这里，因为直接return了）
                    //                 \--上一帧跟踪失败(LOST)--LOST（传不到这里，因为直接return了）
                    // Step 9 如果跟踪成功 或 最近刚刚跟丢，更新速度，清除无效地图点，按需创建关键帧
                    if (bOK || mState == RECENTLY_LOST)
                    {
                        // Update motion model
                        // Step 9.1 更新恒速运动模型 TrackWithMotionModel 中的mVelocity
                        if (!mLastFrame.mTcw.empty() && !mCurrentFrame.mTcw.empty())
                        {
                            cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                            mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                            mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                            // mVelocity = Tcl = Tcw * Twl,表示上一帧到当前帧的变换， 其中 Twl = LastTwc
                            mVelocity = mCurrentFrame.mTcw * LastTwc;
                        }
                        else
                            // 否则速度为空
                            mVelocity = cv::Mat();
                        // 使用IMU积分的位姿显示
                        if (mSensor == ORB_SLAM3::System::IMU_MONOCULAR || mSensor == ORB_SLAM3::System::IMU_STEREO)
                            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

                        // Clean VO matches
                        // Step 9.2 清除观测不到的地图点
                        for (int i = 0; i < mCurrentFrame.N; i++)
                        {
                            ORB_SLAM3::MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                            if (pMP)
                                if (pMP->Observations() < 1)
                                {
                                    mCurrentFrame.mvbOutlier[i] = false;
                                    mCurrentFrame.mvpMapPoints[i] = static_cast<ORB_SLAM3::MapPoint *>(NULL);
                                }
                        }

                        // Delete temporal MapPoints
                        // Step 9.3 清除恒速模型跟踪中 UpdateLastFrame中为当前帧临时添加的MapPoints（仅双目和rgbd）
                        // 上个步骤中只是在当前帧中将这些MapPoints剔除，这里从MapPoints数据库中删除
                        // 临时地图点仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图中
                        for (list<ORB_SLAM3::MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend; lit++)
                        {
                            ORB_SLAM3::MapPoint *pMP = *lit;
                            delete pMP;
                        }
                        // 这里不仅仅是清除mlpTemporalPoints，通过delete pMP还删除了指针指向的MapPoint
                        // 不能够直接执行这个是因为其中存储的都是指针,之前的操作都是为了避免内存泄露
                        mlpTemporalPoints.clear();

                        // 判断是否需要插入关键帧
                        bool bNeedKF = NeedNewKeyFrame();

                        // Check if we need to insert a new keyframe
                        // Step 9.4 根据条件来判断是否插入关键帧
                        // 需要同时满足下面条件1和2
                        // 条件1：bNeedKF=true，需要插入关键帧
                        // 条件2：bOK=true跟踪成功 或 IMU模式下的RECENTLY_LOST模式
                        if (bNeedKF && (bOK || (mState == RECENTLY_LOST && (mSensor == ORB_SLAM3::System::IMU_MONOCULAR || mSensor == ORB_SLAM3::System::IMU_STEREO))))
                            // 创建关键帧，对于双目或RGB-D会产生新的地图点
                            CreateNewKeyFrame();

                        // We allow points with high innovation (considererd outliers by the Huber Function)
                        // pass to the new keyframe, so that bundle adjustment will finally decide
                        // if they are outliers or not. We don't want next frame to estimate its position
                        // with those points so we discard them in the frame. Only has effect if lastframe is tracked
                        // 作者这里说允许在BA中被Huber核函数判断为外点的传入新的关键帧中，让后续的BA来审判他们是不是真正的外点
                        // 但是估计下一帧位姿的时候我们不想用这些外点，所以删掉

                        //  Step 9.5 删除那些在BA中检测为外点的地图点
                        for (int i = 0; i < mCurrentFrame.N; i++)
                        {
                            // 这里第一个条件还要执行判断是因为, 前面的操作中可能删除了其中的地图点
                            if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                                mCurrentFrame.mvpMapPoints[i] = static_cast<ORB_SLAM3::MapPoint *>(NULL);
                        }
                    }

                    // Reset if the camera get lost soon after initialization
                    // Step 10 如果第二阶段跟踪失败，跟踪状态为LOST
                    if (mState == LOST)
                    {
                        // 如果地图中关键帧小于5，重置当前地图，退出当前跟踪
                        if (pCurrentMap->KeyFramesInMap() <= 5)
                        {
                            mpSystem->ResetActiveMap();
                            return;
                        }
                        if ((mSensor == ORB_SLAM3::System::IMU_MONOCULAR) || (mSensor == ORB_SLAM3::System::IMU_STEREO))
                            if (!pCurrentMap->isImuInitialized())
                            {
                                // 如果是IMU模式并且还未进行IMU初始化，重置当前地图，退出当前跟踪
                                ORB_SLAM3::Verbose::PrintMess("Track lost before IMU initialisation, reseting...", ORB_SLAM3::Verbose::VERBOSITY_QUIET);
                                mpSystem->ResetActiveMap();
                                return;
                            }
                        // 如果地图中关键帧超过5 并且 纯视觉模式 或 虽然是IMU模式但是已经完成IMU初始化了，创建新的地图
                        CreateMapInAtlas();
                    }
                    //确保已经设置了参考关键帧
                    if (!mCurrentFrame.mpReferenceKF)
                        mCurrentFrame.mpReferenceKF = mpReferenceKF;
                    // 保存上一帧的数据,当前帧变上一帧
                    mLastFrame = ORB_SLAM3::Frame(mCurrentFrame);
                }
                // 查看到此为止
                // mState的历史变化---上一帧跟踪成功---当前帧跟踪成功---局部地图跟踪成功---OK
                //            \               \              \---局部地图跟踪失败---非OK（IMU时为RECENTLY_LOST）
                //             \               \---当前帧跟踪失败---非OK(地图超过10个关键帧时 RECENTLY_LOST)
                //              \---上一帧跟踪失败(RECENTLY_LOST)---重定位成功---局部地图跟踪成功---OK
                //               \                           \           \---局部地图跟踪失败---LOST
                //                \                           \---重定位失败---LOST（传不到这里，因为直接return了）
                //                 \--上一帧跟踪失败(LOST)--LOST（传不到这里，因为直接return了）
                // last.记录位姿信息，用于轨迹复现
                // Step 11 记录位姿信息，用于最后保存所有的轨迹
                if (mState == OK || mState == RECENTLY_LOST)
                {
                    // Store frame pose information to retrieve the complete camera trajectory afterwards.
                    // Step 11：记录位姿信息，用于最后保存所有的轨迹
                    if (!mCurrentFrame.mTcw.empty())
                    {
                        // 计算相对姿态Tcr = Tcw * Twr, Twr = Trw^-1
                        cv::Mat Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
                        mlRelativeFramePoses.push_back(Tcr);
                        mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
                        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
                        mlbLost.push_back(mState == LOST);
                    }
                    else
                    {
                        // This can happen if tracking is lost
                        // 如果跟踪失败，则相对位姿使用上一次值
                        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
                        mlpReferences.push_back(mlpReferences.back());
                        mlFrameTimes.push_back(mlFrameTimes.back());
                        mlbLost.push_back(mState == LOST);
                    }
                }
                
                thread_recorder->recordSubprocessStop("Track");
            }
        };
    }
}