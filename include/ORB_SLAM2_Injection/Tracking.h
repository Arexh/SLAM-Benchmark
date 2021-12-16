#pragma once

#include "ORB_SLAM2_detailed_comments/include/Tracking.h"

#include "Enums.h"
#include "ThreadRecorder.h"
#include "SystemRecorder.h"

namespace SLAM_Benchmark
{
    namespace ORB_SLAM2_Inject
    {
        class Tracking : public ORB_SLAM2::Tracking
        {
        public:
            Tracking(ORB_SLAM2::System *pSys, ORB_SLAM2::ORBVocabulary *pVoc, ORB_SLAM2::FrameDrawer *pFrameDrawer,
                     ORB_SLAM2::MapDrawer *pMapDrawer, ORB_SLAM2::Map *pMap, ORB_SLAM2::KeyFrameDatabase *pKFDB,
                     const string &strSettingPath, const int sensor)
                : ORB_SLAM2::Tracking(pSys, pVoc, pFrameDrawer, pMapDrawer, pMap, pKFDB, strSettingPath, sensor) {}

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
            cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp) override
            {
                SLAM_Benchmark::ThreadRecorder *thread_recorder = SLAM_Benchmark::SystemRecorder::getInstance(SLAM_Benchmark::SystemName::ORB_SLAM2)->getThreadRecorder("Tracking");

                mImGray = im;

                // Step 1 ：将彩色图像转为灰度图像
                //若图片是3、4通道的，还需要转化成灰度图
                if (mImGray.channels() == 3)
                {
                    if (mbRGB)
                        cvtColor(mImGray, mImGray, CV_RGB2GRAY);
                    else
                        cvtColor(mImGray, mImGray, CV_BGR2GRAY);
                }
                else if (mImGray.channels() == 4)
                {
                    if (mbRGB)
                        cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
                    else
                        cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
                }

                // Step 2 ：构造Frame
                //判断该帧是不是初始化
                thread_recorder->recordSubprocessStart("ORBExtraction");
                if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET) //没有成功初始化的前一个状态就是NO_IMAGES_YET
                    mCurrentFrame = ORB_SLAM2::Frame(
                        mImGray,
                        timestamp,
                        mpIniORBextractor, //初始化ORB特征点提取器会提取2倍的指定特征点数目
                        mpORBVocabulary,
                        mK,
                        mDistCoef,
                        mbf,
                        mThDepth);
                else
                    mCurrentFrame = ORB_SLAM2::Frame(
                        mImGray,
                        timestamp,
                        mpORBextractorLeft, //正常运行的时的ORB特征点提取器，提取指定数目特征点
                        mpORBVocabulary,
                        mK,
                        mDistCoef,
                        mbf,
                        mThDepth);
                thread_recorder->recordSubprocessStop("ORBExtraction");

                // Step 3 ：跟踪
                Track();

                //返回当前帧的位姿
                return mCurrentFrame.mTcw.clone();
            }

            /*
            * @brief Main tracking function. It is independent of the input sensor.
            *
            * track包含两部分：估计运动、跟踪局部地图
            * 
            * Step 1：初始化
            * Step 2：跟踪
            * Step 3：记录位姿信息，用于轨迹复现
            */
            void Track() override
            {
                SLAM_Benchmark::ThreadRecorder *thread_recorder = SLAM_Benchmark::SystemRecorder::getInstance(SLAM_Benchmark::SystemName::ORB_SLAM2)->getThreadRecorder("Tracking");

                // track包含两部分：估计运动、跟踪局部地图

                // mState为tracking的状态，包括 SYSTME_NOT_READY, NO_IMAGE_YET, NOT_INITIALIZED, OK, LOST
                // 如果图像复位过、或者第一次运行，则为NO_IMAGE_YET状态
                if (mState == NO_IMAGES_YET)
                {
                    mState = NOT_INITIALIZED;
                }

                // mLastProcessedState 存储了Tracking最新的状态，用于FrameDrawer中的绘制
                mLastProcessedState = mState;

                // Get Map Mutex -> Map cannot be changed
                // 地图更新时加锁。保证地图不会发生变化
                // 疑问:这样子会不会影响地图的实时更新?
                // 回答：主要耗时在构造帧中特征点的提取和匹配部分,在那个时候地图是没有被上锁的,有足够的时间更新地图
                unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

                thread_recorder->recordSubprocessStart("Track");

                // Step 1：地图初始化
                if (mState == NOT_INITIALIZED)
                {
                    if (mSensor == ORB_SLAM2::System::STEREO || mSensor == ORB_SLAM2::System::RGBD)
                        //双目RGBD相机的初始化共用一个函数
                        StereoInitialization();
                    else
                        //单目初始化
                        MonocularInitialization();

                    //更新帧绘制器中存储的最新状态
                    mpFrameDrawer->Update(this);

                    //这个状态量在上面的初始化函数中被更新
                    if (mState != OK)
                        return;
                }
                else
                {
                    // System is initialized. Track Frame.
                    // bOK为临时变量，用于表示每个函数是否执行成功
                    bool bOK;

                    // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
                    // mbOnlyTracking等于false表示正常SLAM模式（定位+地图更新），mbOnlyTracking等于true表示仅定位模式
                    // tracking 类构造时默认为false。在viewer中有个开关ActivateLocalizationMode，可以控制是否开启mbOnlyTracking
                    if (!mbOnlyTracking)
                    {
                        // Local Mapping is activated. This is the normal behaviour, unless
                        // you explicitly activate the "only tracking" mode.

                        // Step 2：跟踪进入正常SLAM模式，有地图更新
                        // 是否正常跟踪
                        if (mState == OK)
                        {
                            // Local Mapping might have changed some MapPoints tracked in last frame
                            // Step 2.1 检查并更新上一帧被替换的MapPoints
                            // 局部建图线程则可能会对原有的地图点进行替换.在这里进行检查
                            CheckReplacedInLastFrame();

                            // Step 2.2 运动模型是空的或刚完成重定位，跟踪参考关键帧；否则恒速模型跟踪
                            // 第一个条件,如果运动模型为空,说明是刚初始化开始，或者已经跟丢了
                            // 第二个条件,如果当前帧紧紧地跟着在重定位的帧的后面，我们将重定位帧来恢复位姿
                            // mnLastRelocFrameId 上一次重定位的那一帧
                            if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                            {
                                // 用最近的关键帧来跟踪当前的普通帧
                                // 通过BoW的方式在参考帧中找当前帧特征点的匹配点
                                // 优化每个特征点都对应3D点重投影误差即可得到位姿
                                bOK = TrackReferenceKeyFrame();
                            }
                            else
                            {
                                // 用最近的普通帧来跟踪当前的普通帧
                                // 根据恒速模型设定当前帧的初始位姿
                                // 通过投影的方式在参考帧中找当前帧特征点的匹配点
                                // 优化每个特征点所对应3D点的投影误差即可得到位姿
                                bOK = TrackWithMotionModel();
                                if (!bOK)
                                    //根据恒速模型失败了，只能根据参考关键帧来跟踪
                                    bOK = TrackReferenceKeyFrame();
                            }
                        }
                        else
                        {
                            // 如果跟踪状态不成功,那么就只能重定位了
                            // BOW搜索，EPnP求解位姿
                            bOK = Relocalization();
                        }
                    }
                    else
                    {
                        // Localization Mode: Local Mapping is deactivated
                        // Step 2：只进行跟踪tracking，局部地图不工作
                        if (mState == LOST)
                        {
                            // Step 2.1 如果跟丢了，只能重定位
                            bOK = Relocalization();
                        }
                        else
                        {
                            // mbVO是mbOnlyTracking为true时的才有的一个变量
                            // mbVO为false表示此帧匹配了很多的MapPoints，跟踪很正常 (注意有点反直觉)
                            // mbVO为true表明此帧匹配了很少的MapPoints，少于10个，要跪的节奏
                            if (!mbVO)
                            {
                                // Step 2.2 如果跟踪正常，使用恒速模型 或 参考关键帧跟踪
                                // In last frame we tracked enough MapPoints in the map
                                if (!mVelocity.empty())
                                {
                                    bOK = TrackWithMotionModel();
                                    // ? 为了和前面模式统一，这个地方是不是应该加上
                                    // if(!bOK)
                                    //    bOK = TrackReferenceKeyFrame();
                                }
                                else
                                {
                                    // 如果恒速模型不被满足,那么就只能够通过参考关键帧来定位
                                    bOK = TrackReferenceKeyFrame();
                                }
                            }
                            else
                            {
                                // In last frame we tracked mainly "visual odometry" points.
                                // We compute two camera poses, one from motion model and one doing relocalization.
                                // If relocalization is sucessfull we choose that solution, otherwise we retain
                                // the "visual odometry" solution.

                                // mbVO为true，表明此帧匹配了很少（小于10）的地图点，要跪的节奏，既做跟踪又做重定位

                                //MM=Motion Model,通过运动模型进行跟踪的结果
                                bool bOKMM = false;
                                //通过重定位方法来跟踪的结果
                                bool bOKReloc = false;

                                //运动模型中构造的地图点
                                vector<ORB_SLAM2::MapPoint *> vpMPsMM;
                                //在追踪运动模型后发现的外点
                                vector<bool> vbOutMM;
                                //运动模型得到的位姿
                                cv::Mat TcwMM;

                                // Step 2.3 当运动模型有效的时候,根据运动模型计算位姿
                                if (!mVelocity.empty())
                                {
                                    bOKMM = TrackWithMotionModel();

                                    // 将恒速模型跟踪结果暂存到这几个变量中，因为后面重定位会改变这些变量
                                    vpMPsMM = mCurrentFrame.mvpMapPoints;
                                    vbOutMM = mCurrentFrame.mvbOutlier;
                                    TcwMM = mCurrentFrame.mTcw.clone();
                                }

                                // Step 2.4 使用重定位的方法来得到当前帧的位姿
                                bOKReloc = Relocalization();

                                // Step 2.5 根据前面的恒速模型、重定位结果来更新状态
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
                                            //如果这个特征点形成了地图点,并且也不是外点的时候
                                            if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                            {
                                                //增加能观测到该地图点的帧数
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
                                //有一个成功我们就认为执行成功了
                                bOK = bOKReloc || bOKMM;
                            }
                        }
                    }

                    // 将最新的关键帧作为当前帧的参考关键帧
                    mCurrentFrame.mpReferenceKF = mpReferenceKF;

                    // If we have an initial estimation of the camera pose and matching. Track the local map.
                    // Step 3：在跟踪得到当前帧初始姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
                    // 前面只是跟踪一帧得到初始位姿，这里搜索局部关键帧、局部地图点，和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
                    if (!mbOnlyTracking)
                    {
                        if (bOK)
                            bOK = TrackLocalMap();
                    }
                    else
                    {
                        // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
                        // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
                        // the camera we will use the local map again.

                        // 重定位成功
                        if (bOK && !mbVO)
                            bOK = TrackLocalMap();
                    }

                    //根据上面的操作来判断是否追踪成功
                    if (bOK)
                        mState = OK;
                    else
                        mState = LOST;

                    // Step 4：更新显示线程中的图像、特征点、地图点等信息
                    mpFrameDrawer->Update(this);

                    // If tracking were good, check if we insert a keyframe
                    //只有在成功追踪时才考虑生成关键帧的问题
                    if (bOK)
                    {
                        // Update motion model
                        // Step 5：跟踪成功，更新恒速运动模型
                        if (!mLastFrame.mTcw.empty())
                        {
                            // 更新恒速运动模型 TrackWithMotionModel 中的mVelocity
                            cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                            mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                            mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                            // mVelocity = Tcl = Tcw * Twl,表示上一帧到当前帧的变换， 其中 Twl = LastTwc
                            mVelocity = mCurrentFrame.mTcw * LastTwc;
                        }
                        else
                            //否则速度为空
                            mVelocity = cv::Mat();

                        //更新显示中的位姿
                        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

                        // Clean VO matches
                        // Step 6：清除观测不到的地图点
                        for (int i = 0; i < mCurrentFrame.N; i++)
                        {
                            ORB_SLAM2::MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                            if (pMP)
                                if (pMP->Observations() < 1)
                                {
                                    mCurrentFrame.mvbOutlier[i] = false;
                                    mCurrentFrame.mvpMapPoints[i] = static_cast<ORB_SLAM2::MapPoint *>(NULL);
                                }
                        }

                        // Delete temporal MapPoints
                        // Step 7：清除恒速模型跟踪中 UpdateLastFrame中为当前帧临时添加的MapPoints（仅双目和rgbd）
                        // 步骤6中只是在当前帧中将这些MapPoints剔除，这里从MapPoints数据库中删除
                        // 临时地图点仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图中
                        for (list<ORB_SLAM2::MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend; lit++)
                        {
                            ORB_SLAM2::MapPoint *pMP = *lit;
                            delete pMP;
                        }

                        // 这里不仅仅是清除mlpTemporalPoints，通过delete pMP还删除了指针指向的MapPoint
                        // 不能够直接执行这个是因为其中存储的都是指针,之前的操作都是为了避免内存泄露
                        mlpTemporalPoints.clear();

                        // Check if we need to insert a new keyframe
                        // Step 8：检测并插入关键帧，对于双目或RGB-D会产生新的地图点
                        if (NeedNewKeyFrame())
                            CreateNewKeyFrame();

                        // We allow points with high innovation (considererd outliers by the Huber Function)
                        // pass to the new keyframe, so that bundle adjustment will finally decide
                        // if they are outliers or not. We don't want next frame to estimate its position
                        // with those points so we discard them in the frame.
                        // 作者这里说允许在BA中被Huber核函数判断为外点的传入新的关键帧中，让后续的BA来审判他们是不是真正的外点
                        // 但是估计下一帧位姿的时候我们不想用这些外点，所以删掉

                        //  Step 9 删除那些在bundle adjustment中检测为outlier的地图点
                        for (int i = 0; i < mCurrentFrame.N; i++)
                        {
                            // 这里第一个条件还要执行判断是因为, 前面的操作中可能删除了其中的地图点
                            if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                                mCurrentFrame.mvpMapPoints[i] = static_cast<ORB_SLAM2::MapPoint *>(NULL);
                        }
                    }

                    // Reset if the camera get lost soon after initialization
                    // Step 10 如果初始化后不久就跟踪失败，并且relocation也没有搞定，只能重新Reset
                    if (mState == LOST)
                    {
                        //如果地图中的关键帧信息过少的话,直接重新进行初始化了
                        if (mpMap->KeyFramesInMap() <= 5)
                        {
                            cout << "Track lost soon after initialisation, reseting..." << endl;
                            mpSystem->Reset();
                            return;
                        }
                    }

                    //确保已经设置了参考关键帧
                    if (!mCurrentFrame.mpReferenceKF)
                        mCurrentFrame.mpReferenceKF = mpReferenceKF;

                    // 保存上一帧的数据,当前帧变上一帧
                    mLastFrame = ORB_SLAM2::Frame(mCurrentFrame);
                }

                // Store frame pose information to retrieve the complete camera trajectory afterwards.
                // Step 11：记录位姿信息，用于最后保存所有的轨迹
                if (!mCurrentFrame.mTcw.empty())
                {
                    // 计算相对姿态Tcr = Tcw * Twr, Twr = Trw^-1
                    cv::Mat Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
                    //保存各种状态
                    mlRelativeFramePoses.push_back(Tcr);
                    mlpReferences.push_back(mpReferenceKF);
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

                thread_recorder->recordSubprocessStop("Track");

            } // Tracking
        };
    }
}
