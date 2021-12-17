#pragma once

#include "ORB_SLAM3_detailed_comments/include/LoopClosing.h"
#include "ORB_SLAM3_detailed_comments/include/Converter.h"
#include "ORB_SLAM3_detailed_comments/include/Sim3Solver.h"
#include "ORB_SLAM3_detailed_comments/include/ORBmatcher.h"
#include "ORB_SLAM3_detailed_comments/include/G2oTypes.h"
#include "ORB_SLAM3_detailed_comments/include/Optimizer.h"

namespace SLAM_Benchmark
{
    namespace ORB_SLAM3_Inject
    {
        class LoopClosing : public ORB_SLAM3::LoopClosing
        {
        public:
            LoopClosing(ORB_SLAM3::Atlas *pAtlas, ORB_SLAM3::KeyFrameDatabase *pDB, ORB_SLAM3::ORBVocabulary *pVoc, const bool bFixScale) : ORB_SLAM3::LoopClosing(pAtlas, pDB, pVoc, bFixScale) {}

            // 回环线程主函数
            void Run() override
            {
                mbFinished = false;

                // 线程主循环
                while (1)
                {
                    // Check if there are keyframes in the queue
                    // Loopclosing中的关键帧是LocalMapping发送过来的，LocalMapping是Tracking中发过来的
                    // 在LocalMapping中通过 InsertKeyFrame 将关键帧插入闭环检测队列mlpLoopKeyFrameQueue
                    // Step 1 查看闭环检测队列mlpLoopKeyFrameQueue中有没有关键帧进来
                    if (CheckNewKeyFrames())
                    {
                        if (mpLastCurrentKF) // 这部分后续未使用
                        {
                            mpLastCurrentKF->mvpLoopCandKFs.clear();
                            mpLastCurrentKF->mvpMergeCandKFs.clear();
                        }
#ifdef REGISTER_TIMES
                        timeDetectBoW = 0;
                        std::chrono::steady_clock::time_point time_StartDetectBoW = std::chrono::steady_clock::now();
#endif
                        // Step 2 检测有没有共视的区域
                        bool bDetected = NewDetectCommonRegions();
#ifdef REGISTER_TIMES
                        std::chrono::steady_clock::time_point time_EndDetectBoW = std::chrono::steady_clock::now();
                        double timeDetect = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndDetectBoW - time_StartDetectBoW).count();
                        double timeDetectSE3 = timeDetect - timeDetectBoW;

                        if (timeDetectBoW > 0)
                        {
                            vTimeBoW_ms.push_back(timeDetectBoW);
                        }
                        vTimeSE3_ms.push_back(timeDetectSE3);
                        vTimePRTotal_ms.push_back(timeDetect);
#endif

                        if (bDetected)
                        {
                            // Step 3 如果检测到融合（当前关键帧与其他地图有关联）, 则合并地图
                            if (mbMergeDetected)
                            {
                                // 在imu没有初始化就放弃融合
                                if ((mpTracker->mSensor == ORB_SLAM3::System::IMU_MONOCULAR || mpTracker->mSensor == ORB_SLAM3::System::IMU_STEREO) &&
                                    (!mpCurrentKF->GetMap()->isImuInitialized()))
                                {
                                    cout << "IMU is not initilized, merge is aborted" << endl;
                                }
                                // 如果imu成功初始化,或者非imu模式
                                else
                                {
                                    ORB_SLAM3::Verbose::PrintMess("*Merged detected", ORB_SLAM3::Verbose::VERBOSITY_QUIET);
                                    ORB_SLAM3::Verbose::PrintMess("Number of KFs in the current map: " + to_string(mpCurrentKF->GetMap()->KeyFramesInMap()), ORB_SLAM3::Verbose::VERBOSITY_DEBUG);
                                    // 拿到融合帧在自己地图所在坐标系(w2)下的位姿
                                    cv::Mat mTmw = mpMergeMatchedKF->GetPose();
                                    g2o::Sim3 gSmw2(ORB_SLAM3::Converter::toMatrix3d(mTmw.rowRange(0, 3).colRange(0, 3)), ORB_SLAM3::Converter::toVector3d(mTmw.rowRange(0, 3).col(3)), 1.0);
                                    // 拿到当前帧在自己地图所在坐标系(w1)下的位姿
                                    cv::Mat mTcw = mpCurrentKF->GetPose();
                                    g2o::Sim3 gScw1(ORB_SLAM3::Converter::toMatrix3d(mTcw.rowRange(0, 3).colRange(0, 3)), ORB_SLAM3::Converter::toVector3d(mTcw.rowRange(0, 3).col(3)), 1.0);
                                    // 根据共同区域检测时的Sim3结果得到当前帧在w2下的位姿
                                    // mg2oMergeSlw 里存放的是融合候选关键帧所在的世界坐标系w2到当前帧的Sim3位姿
                                    // l = c , w2是融合候选关键帧所在的世界坐标系
                                    g2o::Sim3 gSw2c = mg2oMergeSlw.inverse();
                                    // 这个没有用到 : 融合帧在w1下的位姿
                                    g2o::Sim3 gSw1m = mg2oMergeSlw;

                                    // 记录焊接变换(Sim3) T_w2_w1 , 这个量实际是两个地图坐标系的关系 T_w2_w1 = T_w2_c * T_c_w1
                                    mSold_new = (gSw2c * gScw1);

                                    // 如果是imu模式
                                    if (mpCurrentKF->GetMap()->IsInertial() && mpMergeMatchedKF->GetMap()->IsInertial())
                                    {
                                        // 如果尺度变换太大, 认为累积误差较大，则放弃融合
                                        if (mSold_new.scale() < 0.90 || mSold_new.scale() > 1.1)
                                        {
                                            mpMergeLastCurrentKF->SetErase();
                                            mpMergeMatchedKF->SetErase();
                                            mnMergeNumCoincidences = 0;
                                            mvpMergeMatchedMPs.clear();
                                            mvpMergeMPs.clear();
                                            mnMergeNumNotFound = 0;
                                            mbMergeDetected = false;
                                            ORB_SLAM3::Verbose::PrintMess("scale bad estimated. Abort merging", ORB_SLAM3::Verbose::VERBOSITY_NORMAL);
                                            continue;
                                        }
                                        // If inertial, force only yaw
                                        // 如果是imu模式并且完成了初始化,强制将焊接变换的 roll 和 pitch 设为0
                                        // 可以理解成两个坐标轴都经过了imu初始化，肯定都是水平的
                                        if ((mpTracker->mSensor == ORB_SLAM3::System::IMU_MONOCULAR || mpTracker->mSensor == ORB_SLAM3::System::IMU_STEREO) &&
                                            mpCurrentKF->GetMap()->GetIniertialBA1()) // TODO, maybe with GetIniertialBA1
                                        {
                                            Eigen::Vector3d phi = ORB_SLAM3::LogSO3(mSold_new.rotation().toRotationMatrix());
                                            phi(0) = 0;
                                            phi(1) = 0;
                                            mSold_new = g2o::Sim3(ORB_SLAM3::ExpSO3(phi), mSold_new.translation(), 1.0);
                                        }
                                    }

                                    // 这个变量没有用到
                                    mg2oMergeSmw = gSmw2 * gSw2c * gScw1;

                                    // 更新mg2oMergeScw
                                    mg2oMergeScw = mg2oMergeSlw;

#ifdef REGISTER_TIMES
                                    std::chrono::steady_clock::time_point time_StartMerge = std::chrono::steady_clock::now();
#endif
                                    if (mpTracker->mSensor == ORB_SLAM3::System::IMU_MONOCULAR || mpTracker->mSensor == ORB_SLAM3::System::IMU_STEREO)
                                        // 如果是imu模式,则开启 Visual-Inertial Map Merging
                                        MergeLocal2();
                                    else
                                        // 如果是纯视觉模式,则开启 Visual-Welding Map Merging
                                        MergeLocal();
#ifdef REGISTER_TIMES
                                    std::chrono::steady_clock::time_point time_EndMerge = std::chrono::steady_clock::now();
                                    double timeMerge = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndMerge - time_StartMerge).count();
                                    vTimeMergeTotal_ms.push_back(timeMerge);
#endif
                                }
                                // 记录时间戳
                                vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                                vdPR_MatchedTime.push_back(mpMergeMatchedKF->mTimeStamp);
                                // 标记Place recognition结果为地图融合
                                vnPR_TypeRecogn.push_back(1);

                                // Reset all variables
                                // 重置所有融合相关变量
                                mpMergeLastCurrentKF->SetErase();
                                mpMergeMatchedKF->SetErase();
                                mnMergeNumCoincidences = 0;
                                mvpMergeMatchedMPs.clear();
                                mvpMergeMPs.clear();
                                mnMergeNumNotFound = 0;
                                mbMergeDetected = false;
                                // 重置所有回环相关变量, 说明对与当前帧同时有回环和融合的情况只进行融合
                                if (mbLoopDetected)
                                {
                                    // Reset Loop variables
                                    mpLoopLastCurrentKF->SetErase();
                                    mpLoopMatchedKF->SetErase();
                                    mnLoopNumCoincidences = 0;
                                    mvpLoopMatchedMPs.clear();
                                    mvpLoopMPs.clear();
                                    mnLoopNumNotFound = 0;
                                    mbLoopDetected = false;
                                }
                            }

                            // Step 4 如果(没有检测到融合)检测到回环, 则回环矫正
                            if (mbLoopDetected)
                            {
                                // 标记时间戳
                                vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                                vdPR_MatchedTime.push_back(mpLoopMatchedKF->mTimeStamp);
                                vnPR_TypeRecogn.push_back(0);

                                ORB_SLAM3::Verbose::PrintMess("*Loop detected", ORB_SLAM3::Verbose::VERBOSITY_QUIET);
                                // 更新 mg2oLoopScw
                                mg2oLoopScw = mg2oLoopSlw; //*mvg2oSim3LoopTcw[nCurrentIndex];
                                // 如果是带imu的模式
                                if (mpCurrentKF->GetMap()->IsInertial())
                                {
                                    // 拿到当前关键帧相对于世界坐标系的位姿
                                    cv::Mat Twc = mpCurrentKF->GetPoseInverse();
                                    g2o::Sim3 g2oTwc(ORB_SLAM3::Converter::toMatrix3d(Twc.rowRange(0, 3).colRange(0, 3)), ORB_SLAM3::Converter::toVector3d(Twc.rowRange(0, 3).col(3)), 1.0);

                                    // mg2oLoopScw是通过回环检测的Sim3计算出的回环矫正后的当前关键帧的初始位姿, Twc是当前关键帧回环矫正前的位姿.
                                    // g2oSww_new 可以理解为correction
                                    g2o::Sim3 g2oSww_new = g2oTwc * mg2oLoopScw;

                                    // 拿到 roll ,pitch ,yaw
                                    Eigen::Vector3d phi = ORB_SLAM3::LogSO3(g2oSww_new.rotation().toRotationMatrix());
                                    //cout << "tw2w1: " << g2oSww_new.translation() << endl;
                                    //cout << "Rw2w1: " << g2oSww_new.rotation().toRotationMatrix() << endl;
                                    //cout << "Angle Rw2w1: " << 180*phi/3.14 << endl;
                                    //cout << "scale w2w1: " << g2oSww_new.scale() << endl;
                                    // 这里算是通过imu重力方向验证回环结果, 如果pitch或roll角度偏差稍微有一点大,则回环失败. 对yaw容忍比较大(20度)
                                    if (fabs(phi(0)) < 0.008f && fabs(phi(1)) < 0.008f && fabs(phi(2)) < 0.349f)
                                    {
                                        // 如果是imu模式
                                        if (mpCurrentKF->GetMap()->IsInertial())
                                        {
                                            // If inertial, force only yaw

                                            // 如果是imu模式,强制将焊接变换的的 roll 和 pitch 设为0
                                            if ((mpTracker->mSensor == ORB_SLAM3::System::IMU_MONOCULAR || mpTracker->mSensor == ORB_SLAM3::System::IMU_STEREO) &&
                                                mpCurrentKF->GetMap()->GetIniertialBA2()) // TODO, maybe with GetIniertialBA1
                                            {
                                                phi(0) = 0;
                                                phi(1) = 0;
                                                g2oSww_new = g2o::Sim3(ORB_SLAM3::ExpSO3(phi), g2oSww_new.translation(), 1.0);
                                                mg2oLoopScw = g2oTwc.inverse() * g2oSww_new;
                                            }
                                        }

                                        mvpLoopMapPoints = mvpLoopMPs; //*mvvpLoopMapPoints[nCurrentIndex];

#ifdef REGISTER_TIMES
                                        std::chrono::steady_clock::time_point time_StartLoop = std::chrono::steady_clock::now();
#endif
                                        // 开启回环矫正
                                        CorrectLoop();
#ifdef REGISTER_TIMES
                                        std::chrono::steady_clock::time_point time_EndLoop = std::chrono::steady_clock::now();
                                        double timeLoop = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndLoop - time_StartLoop).count();
                                        vTimeLoopTotal_ms.push_back(timeLoop);
#endif
                                    }
                                    // 如果pitch或roll角度偏差稍微有一点大,或 yaw大与20度则回环失败.
                                    else
                                    {
                                        cout << "BAD LOOP!!!" << endl;
                                    }
                                }
                                // 如果是纯视觉模式
                                else
                                {
                                    mvpLoopMapPoints = mvpLoopMPs;
#ifdef REGISTER_TIMES
                                    std::chrono::steady_clock::time_point time_StartLoop = std::chrono::steady_clock::now();
#endif
                                    CorrectLoop();

#ifdef REGISTER_TIMES
                                    std::chrono::steady_clock::time_point time_EndLoop = std::chrono::steady_clock::now();
                                    double timeLoop = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndLoop - time_StartLoop).count();
                                    vTimeLoopTotal_ms.push_back(timeLoop);
#endif
                                }

                                // Reset all variables
                                // 重置所有的回环变量
                                mpLoopLastCurrentKF->SetErase();
                                mpLoopMatchedKF->SetErase();
                                mnLoopNumCoincidences = 0;
                                mvpLoopMatchedMPs.clear();
                                mvpLoopMPs.clear();
                                mnLoopNumNotFound = 0;
                                mbLoopDetected = false;
                            }
                        }
                        mpLastCurrentKF = mpCurrentKF;
                    }
                    // 查看是否有外部线程请求复位当前线程
                    ResetIfRequested();

                    // 查看外部线程是否有终止当前线程的请求,如果有的话就跳出这个线程的主函数的主循环
                    if (CheckFinish())
                    {
                        // cout << "LC: Finish requested" << endl;
                        break;
                    }

                    usleep(5000);
                }

                //ofstream f_stats;
                //f_stats.open("PlaceRecognition_stats" + mpLocalMapper->strSequence + ".txt");
                //f_stats << "# current_timestamp, matched_timestamp, [0:Loop, 1:Merge]" << endl;
                //f_stats << fixed;
                //for(int i=0; i< vdPR_CurrentTime.size(); ++i)
                //{
                //    f_stats  << 1e9*vdPR_CurrentTime[i] << "," << 1e9*vdPR_MatchedTime[i] << "," << vnPR_TypeRecogn[i] << endl;
                //}

                //f_stats.close();

                SetFinish();
            }

            void CorrectLoop() override
            {
                cout << "Loop detected!" << endl;

                // Send a stop signal to Local Mapping
                // Avoid new keyframes are inserted while correcting the loop
                // Step 0：结束局部地图线程、全局BA，为闭环矫正做准备
                // 请求局部地图停止，防止在回环矫正时局部地图线程中InsertKeyFrame函数插入新的关键帧
                mpLocalMapper->RequestStop();
                mpLocalMapper->EmptyQueue(); // Proccess keyframes in the queue

                // If a Global Bundle Adjustment is running, abort it
                cout << "Request GBA abort" << endl;
                if (isRunningGBA())
                {
                    // 如果有全局BA在运行，终止掉，迎接新的全局BA
                    unique_lock<mutex> lock(mMutexGBA);
                    mbStopGBA = true;
                    // 记录全局BA次数
                    mnFullBAIdx++;

                    if (mpThreadGBA)
                    {
                        cout << "GBA running... Abort!" << endl;
                        mpThreadGBA->detach();
                        delete mpThreadGBA;
                    }
                }

                // Wait until Local Mapping has effectively stopped
                // 一直等到局部地图线程结束再继续
                while (!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }

                // Ensure current keyframe is updated
                // Step 1：根据共视关系更新当前关键帧与其它关键帧之间的连接关系
                // 因为之前闭环检测、计算Sim3中改变了该关键帧的地图点，所以需要更新
                cout << "start updating connections" << endl;
                mpCurrentKF->UpdateConnections();

                // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
                // Step 2：通过位姿传播，得到Sim3优化后，与当前帧相连的关键帧的位姿，以及它们的地图点
                // 当前帧与世界坐标系之间的Sim变换在ComputeSim3函数中已经确定并优化，
                // 通过相对位姿关系，可以确定这些相连的关键帧与世界坐标系之间的Sim3变换

                // 取出当前关键帧及其共视关键帧，称为“当前关键帧组”
                mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
                mvpCurrentConnectedKFs.push_back(mpCurrentKF);

                // CorrectedSim3：存放闭环g2o优化后当前关键帧的共视关键帧的世界坐标系下Sim3 变换
                // NonCorrectedSim3：存放没有矫正的当前关键帧的共视关键帧的世界坐标系下Sim3 变换
                KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
                // 先将mpCurrentKF的Sim3变换存入，认为是准的，所以固定不动
                CorrectedSim3[mpCurrentKF] = mg2oLoopScw;
                // 当前关键帧到世界坐标系下的变换矩阵
                cv::Mat Twc = mpCurrentKF->GetPoseInverse();

                ORB_SLAM3::Map *pLoopMap = mpCurrentKF->GetMap();
                // 对地图点操作
                {
                    // Get Map Mutex
                    // 锁定地图点
                    unique_lock<mutex> lock(pLoopMap->mMutexMapUpdate);

                    const bool bImuInit = pLoopMap->isImuInitialized();
                    // Step 2.1：通过mg2oLoopScw（认为是准的）来进行位姿传播，得到当前关键帧的共视关键帧的世界坐标系下Sim3 位姿（还没有修正）
                    // 遍历"当前关键帧组""
                    for (vector<ORB_SLAM3::KeyFrame *>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end(); vit != vend; vit++)
                    {
                        ORB_SLAM3::KeyFrame *pKFi = *vit;

                        cv::Mat Tiw = pKFi->GetPose();

                        if (pKFi != mpCurrentKF) //跳过当前关键帧，因为当前关键帧的位姿已经在前面优化过了，在这里是参考基准
                        {
                            // 得到当前关键帧 mpCurrentKF 到其共视关键帧 pKFi 的相对变换
                            cv::Mat Tic = Tiw * Twc;
                            cv::Mat Ric = Tic.rowRange(0, 3).colRange(0, 3);
                            cv::Mat tic = Tic.rowRange(0, 3).col(3);

                            // g2oSic：当前关键帧 mpCurrentKF 到其共视关键帧 pKFi 的Sim3 相对变换
                            // 这里是non-correct, 所以scale=1.0
                            g2o::Sim3 g2oSic(ORB_SLAM3::Converter::toMatrix3d(Ric), ORB_SLAM3::Converter::toVector3d(tic), 1.0);
                            // 当前帧的位姿固定不动，其它的关键帧根据相对关系得到Sim3调整的位姿
                            g2o::Sim3 g2oCorrectedSiw = g2oSic * mg2oLoopScw;
                            //Pose corrected with the Sim3 of the loop closure
                            // 存放闭环g2o优化后当前关键帧的共视关键帧的Sim3 位姿
                            CorrectedSim3[pKFi] = g2oCorrectedSiw;
                        }

                        cv::Mat Riw = Tiw.rowRange(0, 3).colRange(0, 3);
                        cv::Mat tiw = Tiw.rowRange(0, 3).col(3);
                        g2o::Sim3 g2oSiw(ORB_SLAM3::Converter::toMatrix3d(Riw), ORB_SLAM3::Converter::toVector3d(tiw), 1.0);
                        //Pose without correction
                        NonCorrectedSim3[pKFi] = g2oSiw;
                    }

                    // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
                    // Step 2.2：得到矫正的当前关键帧的共视关键帧位姿后，修正这些关键帧的地图点
                    // 遍历待矫正的共视关键帧（不包括当前关键帧）
                    for (KeyFrameAndPose::iterator mit = CorrectedSim3.begin(), mend = CorrectedSim3.end(); mit != mend; mit++)
                    {
                        ORB_SLAM3::KeyFrame *pKFi = mit->first;
                        g2o::Sim3 g2oCorrectedSiw = mit->second;
                        g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

                        g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

                        vector<ORB_SLAM3::MapPoint *> vpMPsi = pKFi->GetMapPointMatches();
                        // 遍历待矫正共视关键帧中的每一个地图点
                        for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++)
                        {
                            ORB_SLAM3::MapPoint *pMPi = vpMPsi[iMP];
                            if (!pMPi)
                                continue;
                            if (pMPi->isBad())
                                continue;
                            if (pMPi->mnCorrectedByKF == mpCurrentKF->mnId) // 标记，防止重复矫正
                                continue;

                            // 矫正过程本质上也是基于当前关键帧的优化后的位姿展开的
                            // Project with non-corrected pose and project back with corrected pose
                            // 将该未校正的eigP3Dw先从世界坐标系映射到未校正的pKFi相机坐标系，然后再反映射到校正后的世界坐标系下
                            cv::Mat P3Dw = pMPi->GetWorldPos();
                            // 地图点世界坐标系下坐标
                            Eigen::Matrix<double, 3, 1> eigP3Dw = ORB_SLAM3::Converter::toVector3d(P3Dw);
                            // map(P) 内部做了变换 R*P +t
                            // 下面变换是：eigP3Dw： world →g2oSiw→ i →g2oCorrectedSwi→ world
                            Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                            cv::Mat cvCorrectedP3Dw = ORB_SLAM3::Converter::toCvMat(eigCorrectedP3Dw);
                            pMPi->SetWorldPos(cvCorrectedP3Dw);
                            // 记录矫正该地图点的关键帧id，防止重复
                            pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                            // 记录该地图点所在的关键帧id
                            pMPi->mnCorrectedReference = pKFi->mnId;
                            // 因为地图点更新了，需要更新其平均观测方向以及观测距离范围
                            pMPi->UpdateNormalAndDepth();
                        }

                        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                        // Step 2.3：将共视关键帧的Sim3转换为SE3，根据更新的Sim3，更新关键帧的位姿
                        // 其实是现在已经有了更新后的关键帧组中关键帧的位姿,但是在上面的操作时只是暂时存储到了 KeyFrameAndPose 类型的变量中,还没有写回到关键帧对象中
                        // 调用toRotationMatrix 可以自动归一化旋转矩阵
                        Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
                        Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
                        double s = g2oCorrectedSiw.scale();
                        // cout << "scale for loop-closing: " << s << endl;
                        // 平移向量中包含有尺度信息，还需要用尺度归一化
                        eigt *= (1. / s); //[R t/s;0 1]

                        cv::Mat correctedTiw = ORB_SLAM3::Converter::toCvSE3(eigR, eigt);
                        // 设置矫正后的新的pose
                        pKFi->SetPose(correctedTiw);

                        // Correct velocity according to orientation correction
                        if (bImuInit)
                        {
                            Eigen::Matrix3d Rcor = eigR.transpose() * g2oSiw.rotation().toRotationMatrix();
                            pKFi->SetVelocity(ORB_SLAM3::Converter::toCvMat(Rcor) * pKFi->GetVelocity());
                        }

                        // Make sure connections are updated
                        // Step 2.4：根据共视关系更新当前帧与其它关键帧之间的连接
                        // 地图点的位置改变了,可能会引起共视关系\权值的改变
                        pKFi->UpdateConnections();
                    }
                    // TODO Check this index increasement
                    pLoopMap->IncreaseChangeIndex();

                    // Start Loop Fusion
                    // Update matched map points and replace if duplicated
                    // Step 3：检查当前帧的地图点与经过闭环匹配后该帧的地图点是否存在冲突，对冲突的进行替换或填补
                    // mvpCurrentMatchedPoints 是当前关键帧和闭环关键帧组的所有地图点进行投影得到的匹配点
                    for (size_t i = 0; i < mvpLoopMatchedMPs.size(); i++)
                    {
                        if (mvpLoopMatchedMPs[i])
                        {
                            //取出同一个索引对应的两种地图点，决定是否要替换
                            // 匹配投影得到的地图点
                            ORB_SLAM3::MapPoint *pLoopMP = mvpLoopMatchedMPs[i];
                            // 原来的地图点
                            ORB_SLAM3::MapPoint *pCurMP = mpCurrentKF->GetMapPoint(i);
                            if (pCurMP)
                                // 如果有重复的MapPoint，则用匹配的地图点代替现有的
                                // 因为匹配的地图点是经过一系列操作后比较精确的，现有的地图点很可能有累计误差
                                pCurMP->Replace(pLoopMP);
                            else
                            {
                                // 如果当前帧没有该MapPoint，则直接添加
                                mpCurrentKF->AddMapPoint(pLoopMP, i);
                                pLoopMP->AddObservation(mpCurrentKF, i);
                                pLoopMP->ComputeDistinctiveDescriptors();
                            }
                        }
                    }
                }

                // Project MapPoints observed in the neighborhood of the loop keyframe
                // into the current keyframe and neighbors using corrected poses.
                // Fuse duplications.
                // Step 4：将闭环相连关键帧组mvpLoopMapPoints 投影到当前关键帧组中，进行匹配，融合，新增或替换当前关键帧组中KF的地图点
                // 因为 闭环相连关键帧组mvpLoopMapPoints 在地图中时间比较久经历了多次优化，认为是准确的
                // 而当前关键帧组中的关键帧的地图点是最近新计算的，可能有累积误差
                // CorrectedSim3：存放矫正后当前关键帧的共视关键帧，及其世界坐标系下Sim3 变换
                SearchAndFuse(CorrectedSim3, mvpLoopMapPoints);
                //cout << "LC: end SearchAndFuse" << endl;

                // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
                // Step 5：更新当前关键帧之间的共视相连关系，得到因闭环时MapPoints融合而新得到的连接关系
                // LoopConnections：存储因为闭环地图点调整而新生成的连接关系
                map<ORB_SLAM3::KeyFrame *, set<ORB_SLAM3::KeyFrame *>> LoopConnections;

                // Step 5.1：遍历当前帧相连关键帧组（一级相连）
                for (vector<ORB_SLAM3::KeyFrame *>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end(); vit != vend; vit++)
                {
                    ORB_SLAM3::KeyFrame *pKFi = *vit;
                    // Step 5.2：得到与当前帧相连关键帧的相连关键帧（二级相连）
                    vector<ORB_SLAM3::KeyFrame *> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

                    // Update connections. Detect new links.
                    // Step 5.3：更新一级相连关键帧的连接关系(会把当前关键帧添加进去,因为地图点已经更新和替换了)
                    pKFi->UpdateConnections();
                    // Step 5.4：取出该帧更新后的连接关系
                    LoopConnections[pKFi] = pKFi->GetConnectedKeyFrames();
                    // Step 5.5：从连接关系中去除闭环之前的二级连接关系，剩下的连接就是由闭环得到的连接关系
                    for (vector<ORB_SLAM3::KeyFrame *>::iterator vit_prev = vpPreviousNeighbors.begin(), vend_prev = vpPreviousNeighbors.end(); vit_prev != vend_prev; vit_prev++)
                    {
                        LoopConnections[pKFi].erase(*vit_prev);
                    }
                    // Step 5.6：从连接关系中去除闭环之前的一级连接关系，剩下的连接就是由闭环得到的连接关系
                    for (vector<ORB_SLAM3::KeyFrame *>::iterator vit2 = mvpCurrentConnectedKFs.begin(), vend2 = mvpCurrentConnectedKFs.end(); vit2 != vend2; vit2++)
                    {
                        LoopConnections[pKFi].erase(*vit2);
                    }
                }
                //cout << "LC: end updating covisibility graph" << endl;

                // Optimize graph
                //cout << "start opt essentialgraph" << endl;
                bool bFixedScale = mbFixScale;
                // TODO CHECK; Solo para el monocular inertial
                if (mpTracker->mSensor == ORB_SLAM3::System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                    bFixedScale = false;

                //cout << "Optimize essential graph" << endl;
                if (pLoopMap->IsInertial() && pLoopMap->isImuInitialized())
                {
                    //cout << "With 4DoF" << endl;
                    ORB_SLAM3::Optimizer::OptimizeEssentialGraph4DoF(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections);
                }
                else
                {
                    //cout << "With 7DoF" << endl;
                    // Step 6：进行EssentialGraph优化，LoopConnections是形成闭环后新生成的连接关系，不包括步骤7中当前帧与闭环匹配帧之间的连接关系
                    ORB_SLAM3::Optimizer::OptimizeEssentialGraph(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, bFixedScale);
                }

                //cout << "Optimize essential graph finished" << endl;
                //usleep(5*1000*1000);

                mpAtlas->InformNewBigChange();

                // Add loop edge
                // Step 7：添加当前帧与闭环匹配帧之间的边（这个连接关系不优化）
                // !这两句话应该放在OptimizeEssentialGraph之前，因为OptimizeEssentialGraph的步骤4.2中有优化
                mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);
                mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);

                // Launch a new thread to perform Global Bundle Adjustment (Only if few keyframes, if not it would take too much time)
                // 闭环地图没有imu初始化或者 仅有一个地图且内部关键帧<200时才执行全局BA，否则太慢
                if (!pLoopMap->isImuInitialized() || (pLoopMap->KeyFramesInMap() < 200 && mpAtlas->CountMaps() == 1))
                {
                    // Step 8：新建一个线程用于全局BA优化
                    // OptimizeEssentialGraph只是优化了一些主要关键帧的位姿，这里进行全局BA可以全局优化所有位姿和MapPoints

                    mbRunningGBA = true;
                    mbFinishedGBA = false;
                    mbStopGBA = false;

                    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, pLoopMap, mpCurrentKF->mnId);
                }

                // Loop closed. Release Local Mapping.
                mpLocalMapper->Release();

                mLastLoopKFid = mpCurrentKF->mnId; //TODO old varible, it is not use in the new algorithm
            }

            void RunGlobalBundleAdjustment(ORB_SLAM3::Map *pActiveMap, unsigned long nLoopKF) override
            {
                ORB_SLAM3::Verbose::PrintMess("Starting Global Bundle Adjustment", ORB_SLAM3::Verbose::VERBOSITY_NORMAL);

                const bool bImuInit = pActiveMap->isImuInitialized();

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_StartFGBA = std::chrono::steady_clock::now();
#endif

                if (!bImuInit)
                    ORB_SLAM3::Optimizer::GlobalBundleAdjustemnt(pActiveMap, 10, &mbStopGBA, nLoopKF, false);
                else
                    // 仅有一个地图且内部关键帧<200，并且IMU完成了第一阶段初始化后才会进行下面
                    ORB_SLAM3::Optimizer::FullInertialBA(pActiveMap, 7, false, nLoopKF, &mbStopGBA);

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_StartMapUpdate = std::chrono::steady_clock::now();

                double timeFullGBA = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_StartMapUpdate - time_StartFGBA).count();
                vTimeFullGBA_ms.push_back(timeFullGBA);
#endif

                // 记录GBA已经迭代次数,用来检查全局BA过程是否是因为意外结束的
                int idx = mnFullBAIdx;
                // Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

                // Update all MapPoints and KeyFrames
                // Local Mapping was active during BA, that means that there might be new keyframes
                // not included in the Global BA and they are not consistent with the updated map.
                // We need to propagate the correction through the spanning tree
                {
                    unique_lock<mutex> lock(mMutexGBA);
                    if (idx != mnFullBAIdx)
                        return;

                    if (!bImuInit && pActiveMap->isImuInitialized())
                        return;

                    if (!mbStopGBA)
                    {
                        ORB_SLAM3::Verbose::PrintMess("Global Bundle Adjustment finished", ORB_SLAM3::Verbose::VERBOSITY_NORMAL);
                        ORB_SLAM3::Verbose::PrintMess("Updating map ...", ORB_SLAM3::Verbose::VERBOSITY_NORMAL);

                        mpLocalMapper->RequestStop();
                        // Wait until Local Mapping has effectively stopped

                        while (!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
                        {
                            usleep(1000);
                        }

                        // Get Map Mutex
                        unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);
                        // cout << "LC: Update Map Mutex adquired" << endl;

                        //pActiveMap->PrintEssentialGraph();
                        // Correct keyframes starting at map first keyframe
                        list<ORB_SLAM3::KeyFrame *> lpKFtoCheck(pActiveMap->mvpKeyFrameOrigins.begin(), pActiveMap->mvpKeyFrameOrigins.end());

                        while (!lpKFtoCheck.empty())
                        {
                            ORB_SLAM3::KeyFrame *pKF = lpKFtoCheck.front();
                            const set<ORB_SLAM3::KeyFrame *> sChilds = pKF->GetChilds();
                            //cout << "---Updating KF " << pKF->mnId << " with " << sChilds.size() << " childs" << endl;
                            //cout << " KF mnBAGlobalForKF: " << pKF->mnBAGlobalForKF << endl;
                            cv::Mat Twc = pKF->GetPoseInverse();
                            //cout << "Twc: " << Twc << endl;
                            //cout << "GBA: Correct KeyFrames" << endl;
                            for (set<ORB_SLAM3::KeyFrame *>::const_iterator sit = sChilds.begin(); sit != sChilds.end(); sit++)
                            {
                                ORB_SLAM3::KeyFrame *pChild = *sit;
                                if (!pChild || pChild->isBad())
                                    continue;

                                if (pChild->mnBAGlobalForKF != nLoopKF)
                                {
                                    //cout << "++++New child with flag " << pChild->mnBAGlobalForKF << "; LoopKF: " << nLoopKF << endl;
                                    //cout << " child id: " << pChild->mnId << endl;
                                    cv::Mat Tchildc = pChild->GetPose() * Twc;
                                    //cout << "Child pose: " << Tchildc << endl;
                                    //cout << "pKF->mTcwGBA: " << pKF->mTcwGBA << endl;
                                    pChild->mTcwGBA = Tchildc * pKF->mTcwGBA; //*Tcorc*pKF->mTcwGBA;

                                    cv::Mat Rcor = pChild->mTcwGBA.rowRange(0, 3).colRange(0, 3).t() * pChild->GetRotation();
                                    if (!pChild->GetVelocity().empty())
                                    {
                                        //cout << "Child velocity: " << pChild->GetVelocity() << endl;
                                        pChild->mVwbGBA = Rcor * pChild->GetVelocity();
                                    }
                                    else
                                        ORB_SLAM3::Verbose::PrintMess("Child velocity empty!! ", ORB_SLAM3::Verbose::VERBOSITY_NORMAL);

                                    //cout << "Child bias: " << pChild->GetImuBias() << endl;
                                    pChild->mBiasGBA = pChild->GetImuBias();

                                    pChild->mnBAGlobalForKF = nLoopKF;
                                }
                                lpKFtoCheck.push_back(pChild);
                            }

                            //cout << "-------Update pose" << endl;
                            pKF->mTcwBefGBA = pKF->GetPose();
                            //cout << "pKF->mTcwBefGBA: " << pKF->mTcwBefGBA << endl;
                            pKF->SetPose(pKF->mTcwGBA);
                            /*cv::Mat Tco_cn = pKF->mTcwBefGBA * pKF->mTcwGBA.inv();
                cv::Vec3d trasl = Tco_cn.rowRange(0,3).col(3);
                double dist = cv::norm(trasl);
                cout << "GBA: KF " << pKF->mnId << " had been moved " << dist << " meters" << endl;
                double desvX = 0;
                double desvY = 0;
                double desvZ = 0;
                if(pKF->mbHasHessian)
                {
                    cv::Mat hessianInv = pKF->mHessianPose.inv();

                    double covX = hessianInv.at<double>(3,3);
                    desvX = std::sqrt(covX);
                    double covY = hessianInv.at<double>(4,4);
                    desvY = std::sqrt(covY);
                    double covZ = hessianInv.at<double>(5,5);
                    desvZ = std::sqrt(covZ);
                    pKF->mbHasHessian = false;
                }
                if(dist > 1)
                {
                    cout << "--To much distance correction: It has " << pKF->GetConnectedKeyFrames().size() << " connected KFs" << endl;
                    cout << "--It has " << pKF->GetCovisiblesByWeight(80).size() << " connected KF with 80 common matches or more" << endl;
                    cout << "--It has " << pKF->GetCovisiblesByWeight(50).size() << " connected KF with 50 common matches or more" << endl;
                    cout << "--It has " << pKF->GetCovisiblesByWeight(20).size() << " connected KF with 20 common matches or more" << endl;

                    cout << "--STD in meters(x, y, z): " << desvX << ", " << desvY << ", " << desvZ << endl;


                    string strNameFile = pKF->mNameFile;
                    cv::Mat imLeft = cv::imread(strNameFile, CV_LOAD_IMAGE_UNCHANGED);

                    cv::cvtColor(imLeft, imLeft, CV_GRAY2BGR);

                    vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();
                    int num_MPs = 0;
                    for(int i=0; i<vpMapPointsKF.size(); ++i)
                    {
                        if(!vpMapPointsKF[i] || vpMapPointsKF[i]->isBad())
                        {
                            continue;
                        }
                        num_MPs += 1;
                        string strNumOBs = to_string(vpMapPointsKF[i]->Observations());
                        cv::circle(imLeft, pKF->mvKeys[i].pt, 2, cv::Scalar(0, 255, 0));
                        cv::putText(imLeft, strNumOBs, pKF->mvKeys[i].pt, CV_FONT_HERSHEY_DUPLEX, 1, cv::Scalar(255, 0, 0));
                    }
                    cout << "--It has " << num_MPs << " MPs matched in the map" << endl;

                    string namefile = "./test_GBA/GBA_" + to_string(nLoopKF) + "_KF" + to_string(pKF->mnId) +"_D" + to_string(dist) +".png";
                    cv::imwrite(namefile, imLeft);
                }*/

                            if (pKF->bImu)
                            {
                                //cout << "-------Update inertial values" << endl;
                                pKF->mVwbBefGBA = pKF->GetVelocity();
                                if (pKF->mVwbGBA.empty())
                                    ORB_SLAM3::Verbose::PrintMess("pKF->mVwbGBA is empty", ORB_SLAM3::Verbose::VERBOSITY_NORMAL);

                                assert(!pKF->mVwbGBA.empty());
                                pKF->SetVelocity(pKF->mVwbGBA);
                                pKF->SetNewBias(pKF->mBiasGBA);
                            }

                            lpKFtoCheck.pop_front();
                        }

                        //cout << "GBA: Correct MapPoints" << endl;
                        // Correct MapPoints
                        const vector<ORB_SLAM3::MapPoint *> vpMPs = pActiveMap->GetAllMapPoints();

                        // 遍历每一个地图点
                        for (size_t i = 0; i < vpMPs.size(); i++)
                        {
                            ORB_SLAM3::MapPoint *pMP = vpMPs[i];

                            if (pMP->isBad())
                                continue;

                            // NOTICE 并不是所有的地图点都会直接参与到全局BA优化中,但是大部分的地图点需要根据全局BA优化后的结果来重新纠正自己的位姿
                            // 如果这个地图点直接参与到了全局BA优化的过程,那么就直接重新设置器位姿即可
                            if (pMP->mnBAGlobalForKF == nLoopKF)
                            {
                                // If optimized by Global BA, just update
                                pMP->SetWorldPos(pMP->mPosGBA);
                            }
                            else // 如故这个地图点并没有直接参与到全局BA优化的过程中,那么就使用器参考关键帧的新位姿来优化自己的位姿
                            {
                                // Update according to the correction of its reference keyframe
                                ORB_SLAM3::KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();

                                // 说明这个关键帧，在前面的过程中也没有因为“当前关键帧”得到全局BA优化
                                //? 可是,为什么会出现这种情况呢? 难道是因为这个地图点的参考关键帧设置成为了bad?
                                if (pRefKF->mnBAGlobalForKF != nLoopKF)
                                    continue;

                                if (pRefKF->mTcwBefGBA.empty())
                                    continue;

                                // Map to non-corrected camera
                                cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0, 3).colRange(0, 3);
                                cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0, 3).col(3);
                                // 转换到其参考关键帧相机坐标系下的坐标
                                cv::Mat Xc = Rcw * pMP->GetWorldPos() + tcw;

                                // Backproject using corrected camera
                                // 然后使用已经纠正过的参考关键帧的位姿,再将该地图点变换到世界坐标系下
                                cv::Mat Twc = pRefKF->GetPoseInverse();
                                cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
                                cv::Mat twc = Twc.rowRange(0, 3).col(3);

                                pMP->SetWorldPos(Rwc * Xc + twc);
                            }
                        }

                        pActiveMap->InformNewBigChange();
                        pActiveMap->IncreaseChangeIndex();

                        // TODO Check this update
                        // mpTracker->UpdateFrameIMU(1.0f, mpTracker->GetLastKeyFrame()->GetImuBias(), mpTracker->GetLastKeyFrame());

                        mpLocalMapper->Release();

                        ORB_SLAM3::Verbose::PrintMess("Map updated!", ORB_SLAM3::Verbose::VERBOSITY_NORMAL);
                    }

                    mbFinishedGBA = true;
                    mbRunningGBA = false;
                }

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndMapUpdate = std::chrono::steady_clock::now();

                double timeMapUpdate = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndMapUpdate - time_StartMapUpdate).count();
                vTimeMapUpdate_ms.push_back(timeMapUpdate);

                double timeGBA = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(time_EndMapUpdate - time_StartFGBA).count();
                vTimeGBATotal_ms.push_back(timeGBA);
#endif
            }
        };
    }
}