#pragma once

#include "ORB_SLAM2_detailed_comments/include/LoopClosing.h"
#include "ORB_SLAM2_detailed_comments/include/Optimizer.h"
#include "ORB_SLAM2_detailed_comments/include/Converter.h"
#include "ORB_SLAM2_detailed_comments/include/KeyFrame.h"

#include "ThreadRecorder.h"
#include "SystemRecorder.h"

namespace SLAM_Benchmark
{
    namespace ORB_SLAM2_Inject
    {
        class LoopClosing : public ORB_SLAM2::LoopClosing
        {
        public:
            LoopClosing(ORB_SLAM2::Map *pMap, ORB_SLAM2::KeyFrameDatabase *pDB, ORB_SLAM2::ORBVocabulary *pVoc, const bool bFixScale) : ORB_SLAM2::LoopClosing(pMap, pDB, pVoc, bFixScale) {}

            // 回环线程主函数
            void Run() override
            {
                SLAM_Benchmark::ThreadRecorder *thread_recorder = SLAM_Benchmark::SystemRecorder::getInstance(SLAM_Benchmark::SystemName::ORB_SLAM2)->getThreadRecorder("LoopClosing");

                thread_recorder->recordThreadCreate();

                mbFinished = false;

                // 线程主循环
                while (1)
                {
                    thread_recorder->recordThreadProcessStart();

                    // Check if there are keyframes in the queue
                    // Loopclosing中的关键帧是LocalMapping发送过来的，LocalMapping是Tracking中发过来的
                    // 在LocalMapping中通过 InsertKeyFrame 将关键帧插入闭环检测队列mlpLoopKeyFrameQueue
                    // Step 1 查看闭环检测队列mlpLoopKeyFrameQueue中有没有关键帧进来
                    if (CheckNewKeyFrames())
                    {
                        // Detect loop candidates and check covisibility consistency
                        thread_recorder->recordSubprocessStart("DetectLoop");
                        bool detected = DetectLoop();
                        thread_recorder->recordSubprocessStop("DetectLoop");
                        if (detected)
                        {
                            // Compute similarity transformation [sR|t]
                            // In the stereo/RGBD case s=1
                            thread_recorder->recordSubprocessStart("ComputeSim3");
                            bool sim3 = ComputeSim3();
                            thread_recorder->recordSubprocessStop("ComputeSim3");
                            if (sim3)
                            {
                                // Perform loop fusion and pose graph optimization
                                CorrectLoop();
                            }
                        }
                    }

                    // 查看是否有外部线程请求复位当前线程
                    ResetIfRequested();

                    thread_recorder->recordThreadProcessStop();

                    // 查看外部线程是否有终止当前线程的请求,如果有的话就跳出这个线程的主函数的主循环
                    if (CheckFinish())
                        break;

                    //usleep(5000);
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }

                // 运行到这里说明有外部线程请求终止当前线程,在这个函数中执行终止当前线程的一些操作
                SetFinish();

                thread_recorder->recordThreadDestory();
            }

            void CorrectLoop() override
            {
                SLAM_Benchmark::ThreadRecorder *thread_recorder = SLAM_Benchmark::SystemRecorder::getInstance(SLAM_Benchmark::SystemName::ORB_SLAM2)->getThreadRecorder("LoopClosing");

                cout << "Loop detected!" << endl;
                // Step 0：结束局部地图线程、全局BA，为闭环矫正做准备
                // Step 1：根据共视关系更新当前帧与其它关键帧之间的连接
                // Step 2：通过位姿传播，得到Sim3优化后，与当前帧相连的关键帧的位姿，以及它们的MapPoints
                // Step 3：检查当前帧的MapPoints与闭环匹配帧的MapPoints是否存在冲突，对冲突的MapPoints进行替换或填补
                // Step 4：通过将闭环时相连关键帧的mvpLoopMapPoints投影到这些关键帧中，进行MapPoints检查与替换
                // Step 5：更新当前关键帧之间的共视相连关系，得到因闭环时MapPoints融合而新得到的连接关系
                // Step 6：进行EssentialGraph优化，LoopConnections是形成闭环后新生成的连接关系，不包括步骤7中当前帧与闭环匹配帧之间的连接关系
                // Step 7：添加当前帧与闭环匹配帧之间的边（这个连接关系不优化）
                // Step 8：新建一个线程用于全局BA优化

                // g2oSic： 当前关键帧 mpCurrentKF 到其共视关键帧 pKFi 的Sim3 相对变换
                // mg2oScw: 世界坐标系到当前关键帧的 Sim3 变换
                // g2oCorrectedSiw：世界坐标系到当前关键帧共视关键帧的Sim3 变换

                // Send a stop signal to Local Mapping
                // Avoid new keyframes are inserted while correcting the loop
                // Step 0：结束局部地图线程、全局BA，为闭环矫正做准备
                // 请求局部地图停止，防止在回环矫正时局部地图线程中InsertKeyFrame函数插入新的关键帧
                mpLocalMapper->RequestStop();

                if (isRunningGBA())
                {
                    // 如果有全局BA在运行，终止掉，迎接新的全局BA
                    unique_lock<mutex> lock(mMutexGBA);
                    mbStopGBA = true;
                    // 记录全局BA次数
                    mnFullBAIdx++;
                    if (mpThreadGBA)
                    {
                        // 停止全局BA线程
                        mpThreadGBA->detach();
                        delete mpThreadGBA;
                    }
                }

                // Wait until Local Mapping has effectively stopped
                // 一直等到局部地图线程结束再继续
                while (!mpLocalMapper->isStopped())
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }

                thread_recorder->recordSubprocessStart("SearchAndFuse");
                // Ensure current keyframe is updated
                // Step 1：根据共视关系更新当前关键帧与其它关键帧之间的连接关系
                // 因为之前闭环检测、计算Sim3中改变了该关键帧的地图点，所以需要更新
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
                CorrectedSim3[mpCurrentKF] = mg2oScw;
                // 当前关键帧到世界坐标系下的变换矩阵
                cv::Mat Twc = mpCurrentKF->GetPoseInverse();

                // 对地图点操作
                {
                    // Get Map Mutex
                    // 锁定地图点
                    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

                    // Step 2.1：通过mg2oScw（认为是准的）来进行位姿传播，得到当前关键帧的共视关键帧的世界坐标系下Sim3 位姿
                    // 遍历"当前关键帧组""
                    for (vector<ORB_SLAM2::KeyFrame *>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end(); vit != vend; vit++)
                    {
                        ORB_SLAM2::KeyFrame *pKFi = *vit;
                        cv::Mat Tiw = pKFi->GetPose();
                        if (pKFi != mpCurrentKF) //跳过当前关键帧，因为当前关键帧的位姿已经在前面优化过了，在这里是参考基准
                        {
                            // 得到当前关键帧 mpCurrentKF 到其共视关键帧 pKFi 的相对变换
                            cv::Mat Tic = Tiw * Twc;
                            cv::Mat Ric = Tic.rowRange(0, 3).colRange(0, 3);
                            cv::Mat tic = Tic.rowRange(0, 3).col(3);

                            // g2oSic：当前关键帧 mpCurrentKF 到其共视关键帧 pKFi 的Sim3 相对变换
                            // 这里是non-correct, 所以scale=1.0
                            g2o::Sim3 g2oSic(ORB_SLAM2::Converter::toMatrix3d(Ric), ORB_SLAM2::Converter::toVector3d(tic), 1.0);
                            // 当前帧的位姿固定不动，其它的关键帧根据相对关系得到Sim3调整的位姿
                            g2o::Sim3 g2oCorrectedSiw = g2oSic * mg2oScw;
                            // Pose corrected with the Sim3 of the loop closure
                            // 存放闭环g2o优化后当前关键帧的共视关键帧的Sim3 位姿
                            CorrectedSim3[pKFi] = g2oCorrectedSiw;
                        }

                        cv::Mat Riw = Tiw.rowRange(0, 3).colRange(0, 3);
                        cv::Mat tiw = Tiw.rowRange(0, 3).col(3);
                        g2o::Sim3 g2oSiw(ORB_SLAM2::Converter::toMatrix3d(Riw), ORB_SLAM2::Converter::toVector3d(tiw), 1.0);
                        // Pose without correction
                        // 存放没有矫正的当前关键帧的共视关键帧的Sim3变换
                        NonCorrectedSim3[pKFi] = g2oSiw;
                    }

                    // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
                    // Step 2.2：得到矫正的当前关键帧的共视关键帧位姿后，修正这些共视关键帧的地图点
                    // 遍历待矫正的共视关键帧（不包括当前关键帧）
                    for (KeyFrameAndPose::iterator mit = CorrectedSim3.begin(), mend = CorrectedSim3.end(); mit != mend; mit++)
                    {
                        // 取出当前关键帧连接关键帧
                        ORB_SLAM2::KeyFrame *pKFi = mit->first;
                        // 取出经过位姿传播后的Sim3变换
                        g2o::Sim3 g2oCorrectedSiw = mit->second;
                        g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();
                        // 取出未经过位姿传播的Sim3变换
                        g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

                        vector<ORB_SLAM2::MapPoint *> vpMPsi = pKFi->GetMapPointMatches();
                        // 遍历待矫正共视关键帧中的每一个地图点
                        for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++)
                        {
                            ORB_SLAM2::MapPoint *pMPi = vpMPsi[iMP];
                            // 跳过无效的地图点
                            if (!pMPi)
                                continue;
                            if (pMPi->isBad())
                                continue;
                            // 标记，防止重复矫正
                            if (pMPi->mnCorrectedByKF == mpCurrentKF->mnId)
                                continue;

                            // 矫正过程本质上也是基于当前关键帧的优化后的位姿展开的
                            // Project with non-corrected pose and project back with corrected pose
                            // 将该未校正的eigP3Dw先从世界坐标系映射到未校正的pKFi相机坐标系，然后再反映射到校正后的世界坐标系下
                            cv::Mat P3Dw = pMPi->GetWorldPos();
                            // 地图点世界坐标系下坐标
                            Eigen::Matrix<double, 3, 1> eigP3Dw = ORB_SLAM2::Converter::toVector3d(P3Dw);
                            // map(P) 内部做了相似变换 s*R*P +t
                            // 下面变换是：eigP3Dw： world →g2oSiw→ i →g2oCorrectedSwi→ world
                            Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                            cv::Mat cvCorrectedP3Dw = ORB_SLAM2::Converter::toCvMat(eigCorrectedP3Dw);
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
                        // 平移向量中包含有尺度信息，还需要用尺度归一化
                        eigt *= (1. / s);

                        cv::Mat correctedTiw = ORB_SLAM2::Converter::toCvSE3(eigR, eigt);
                        // 设置矫正后的新的pose
                        pKFi->SetPose(correctedTiw);

                        // Make sure connections are updated
                        // Step 2.4：根据共视关系更新当前帧与其它关键帧之间的连接
                        // 地图点的位置改变了,可能会引起共视关系\权值的改变
                        pKFi->UpdateConnections();
                    }

                    // Start Loop Fusion
                    // Update matched map points and replace if duplicated
                    // Step 3：检查当前帧的地图点与经过闭环匹配后该帧的地图点是否存在冲突，对冲突的进行替换或填补
                    // mvpCurrentMatchedPoints 是当前关键帧和闭环关键帧组的所有地图点进行投影得到的匹配点
                    for (size_t i = 0; i < mvpCurrentMatchedPoints.size(); i++)
                    {
                        if (mvpCurrentMatchedPoints[i])
                        {
                            //取出同一个索引对应的两种地图点，决定是否要替换
                            // 匹配投影得到的地图点
                            ORB_SLAM2::MapPoint *pLoopMP = mvpCurrentMatchedPoints[i];
                            // 原来的地图点
                            ORB_SLAM2::MapPoint *pCurMP = mpCurrentKF->GetMapPoint(i);
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
                SearchAndFuse(CorrectedSim3);

                // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
                // Step 5：更新当前关键帧组之间的两级共视相连关系，得到因闭环时地图点融合而新得到的连接关系
                // LoopConnections：存储因为闭环地图点调整而新生成的连接关系
                map<ORB_SLAM2::KeyFrame *, set<ORB_SLAM2::KeyFrame *>> LoopConnections;

                // Step 5.1：遍历当前帧相连关键帧组（一级相连）
                for (vector<ORB_SLAM2::KeyFrame *>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end(); vit != vend; vit++)
                {
                    ORB_SLAM2::KeyFrame *pKFi = *vit;
                    // Step 5.2：得到与当前帧相连关键帧的相连关键帧（二级相连）
                    vector<ORB_SLAM2::KeyFrame *> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

                    // Update connections. Detect new links.
                    // Step 5.3：更新一级相连关键帧的连接关系(会把当前关键帧添加进去,因为地图点已经更新和替换了)
                    pKFi->UpdateConnections();
                    // Step 5.4：取出该帧更新后的连接关系
                    LoopConnections[pKFi] = pKFi->GetConnectedKeyFrames();
                    // Step 5.5：从连接关系中去除闭环之前的二级连接关系，剩下的连接就是由闭环得到的连接关系
                    for (vector<ORB_SLAM2::KeyFrame *>::iterator vit_prev = vpPreviousNeighbors.begin(), vend_prev = vpPreviousNeighbors.end(); vit_prev != vend_prev; vit_prev++)
                    {
                        LoopConnections[pKFi].erase(*vit_prev);
                    }
                    // Step 5.6：从连接关系中去除闭环之前的一级连接关系，剩下的连接就是由闭环得到的连接关系
                    for (vector<ORB_SLAM2::KeyFrame *>::iterator vit2 = mvpCurrentConnectedKFs.begin(), vend2 = mvpCurrentConnectedKFs.end(); vit2 != vend2; vit2++)
                    {
                        LoopConnections[pKFi].erase(*vit2);
                    }
                }
                thread_recorder->recordSubprocessStop("SearchAndFuse");

                thread_recorder->recordSubprocessStart("OptimizeEssentialGraph");
                // Optimize graph
                // Step 6：进行本质图优化，优化本质图中所有关键帧的位姿和地图点
                // LoopConnections是形成闭环后新生成的连接关系，不包括步骤7中当前帧与闭环匹配帧之间的连接关系
                ORB_SLAM2::Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);
                thread_recorder->recordSubprocessStop("OptimizeEssentialGraph");

                // Add loop edge
                // Step 7：添加当前帧与闭环匹配帧之间的边（这个连接关系不优化）
                // !这两句话应该放在OptimizeEssentialGraph之前，因为OptimizeEssentialGraph的步骤4.2中有优化
                mpMatchedKF->AddLoopEdge(mpCurrentKF);
                mpCurrentKF->AddLoopEdge(mpMatchedKF);

                // Launch a new thread to perform Global Bundle Adjustment
                // Step 8：新建一个线程用于全局BA优化
                // OptimizeEssentialGraph只是优化了一些主要关键帧的位姿，这里进行全局BA可以全局优化所有位姿和MapPoints
                mbRunningGBA = true;
                mbFinishedGBA = false;
                mbStopGBA = false;
                mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, mpCurrentKF->mnId);

                // Loop closed. Release Local Mapping.
                mpLocalMapper->Release();

                cout << "Loop Closed!" << endl;

                mLastLoopKFid = mpCurrentKF->mnId;
            }

            /**
             * @brief 全局BA线程,这个是这个线程的主函数
             * 
             * @param[in] nLoopKF 看上去是闭环关键帧id,但是在调用的时候给的其实是当前关键帧的id
             */
            void RunGlobalBundleAdjustment(unsigned long nLoopKF)
            {
                SLAM_Benchmark::ThreadRecorder *thread_recorder = SLAM_Benchmark::SystemRecorder::getInstance(SLAM_Benchmark::SystemName::ORB_SLAM2)->getThreadRecorder("BundleAdjustment");

                cout << "Starting Global Bundle Adjustment" << endl;

                // 记录GBA已经迭代次数,用来检查全局BA过程是否是因为意外结束的
                int idx = mnFullBAIdx;
                // mbStopGBA直接传引用过去了,这样当有外部请求的时候这个优化函数能够及时响应并且结束掉
                // 提问:进行完这个过程后我们能够获得哪些信息?
                // 回答：能够得到全部关键帧优化后的位姿,以及优化后的地图点

                thread_recorder->recordSubprocessStart("GlobalBundleAdjustemnt");
                // Step 1 执行全局BA，优化所有的关键帧位姿和地图中地图点
                ORB_SLAM2::Optimizer::GlobalBundleAdjustemnt(mpMap,      // 地图点对象
                                                             10,         // 迭代次数
                                                             &mbStopGBA, // 外界控制 GBA 停止的标志
                                                             nLoopKF,    // 形成了闭环的当前关键帧的id
                                                             false);     // 不使用鲁棒核函数
                thread_recorder->recordSubprocessStop("GlobalBundleAdjustemnt");

                // Update all MapPoints and KeyFrames
                // Local Mapping was active during BA, that means that there might be new keyframes
                // not included in the Global BA and they are not consistent with the updated map.
                // We need to propagate the correction through the spanning tree
                // 更新所有的地图点和关键帧
                // 在global BA过程中local mapping线程仍然在工作，这意味着在global BA时可能有新的关键帧产生，但是并未包括在GBA里，
                // 所以和更新后的地图并不连续。需要通过spanning tree来传播
                {
                    unique_lock<mutex> lock(mMutexGBA);
                    // 如果全局BA过程是因为意外结束的,那么直接退出GBA
                    if (idx != mnFullBAIdx)
                        return;

                    // 如果当前GBA没有中断请求，更新位姿和地图点
                    // 这里和上面那句话的功能还有些不同,因为如果一次全局优化被中断,往往意味又要重新开启一个新的全局BA;为了中断当前正在执行的优化过程mbStopGBA将会被置位,同时会有一定的时间
                    // 使得该线程进行响应;而在开启一个新的全局优化进程之前 mbStopGBA 将会被置为False
                    // 因此,如果被强行中断的线程退出时已经有新的线程启动了,mbStopGBA=false,为了避免进行后面的程序,所以有了上面的程序;
                    // 而如果被强行中断的线程退出时新的线程还没有启动,那么上面的条件就不起作用了(虽然概率很小,前面的程序中mbStopGBA置位后很快mnFullBAIdx就++了,保险起见),所以这里要再判断一次
                    if (!mbStopGBA)
                    {
                        cout << "Global Bundle Adjustment finished" << endl;
                        cout << "Updating map ..." << endl;
                        mpLocalMapper->RequestStop();

                        // Wait until Local Mapping has effectively stopped
                        // 等待直到local mapping结束才会继续后续操作
                        while (!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
                        {
                            //usleep(1000);
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        }

                        // Get Map Mutex
                        // 后续要更新地图所以要上锁
                        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

                        thread_recorder->recordSubprocessStart("MapUpdate");

                        // Correct keyframes starting at map first keyframe
                        // 从第一个关键帧开始矫正关键帧。刚开始只保存了初始化第一个关键帧
                        list<ORB_SLAM2::KeyFrame *> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(), mpMap->mvpKeyFrameOrigins.end());

                        // 问：GBA里锁住第一个关键帧位姿没有优化，其对应的pKF->mTcwGBA是不变的吧？那后面调整位姿的意义何在？
                        // 回答：注意在前面essential graph BA里只锁住了回环帧，没有锁定第1个初始化关键帧位姿。所以第1个初始化关键帧位姿已经更新了
                        // 在GBA里锁住第一个关键帧位姿没有优化，其对应的pKF->mTcwGBA应该是essential BA结果，在这里统一更新了
                        // Step 2 遍历并更新全局地图中的所有spanning tree中的关键帧
                        while (!lpKFtoCheck.empty())
                        {
                            ORB_SLAM2::KeyFrame *pKF = lpKFtoCheck.front();
                            const set<ORB_SLAM2::KeyFrame *> sChilds = pKF->GetChilds();
                            cv::Mat Twc = pKF->GetPoseInverse();
                            // 遍历当前关键帧的子关键帧
                            for (set<ORB_SLAM2::KeyFrame *>::const_iterator sit = sChilds.begin(); sit != sChilds.end(); sit++)
                            {
                                ORB_SLAM2::KeyFrame *pChild = *sit;
                                // 记录避免重复
                                if (pChild->mnBAGlobalForKF != nLoopKF)
                                {
                                    // 从父关键帧到当前子关键帧的位姿变换 T_child_farther
                                    cv::Mat Tchildc = pChild->GetPose() * Twc;
                                    // 再利用优化后的父关键帧的位姿，转换到世界坐标系下，相当于更新了子关键帧的位姿
                                    // 这种最小生成树中除了根节点，其他的节点都会作为其他关键帧的子节点，这样做可以使得最终所有的关键帧都得到了优化
                                    pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;
                                    // 做个标记，避免重复
                                    pChild->mnBAGlobalForKF = nLoopKF;
                                }
                                lpKFtoCheck.push_back(pChild);
                            }
                            // 记录未矫正的关键帧的位姿
                            pKF->mTcwBefGBA = pKF->GetPose();
                            // 记录已经矫正的关键帧的位姿
                            pKF->SetPose(pKF->mTcwGBA);
                            // 从列表中移除
                            lpKFtoCheck.pop_front();
                        }

                        // Correct MapPoints
                        const vector<ORB_SLAM2::MapPoint *> vpMPs = mpMap->GetAllMapPoints();

                        // Step 3 遍历每一个地图点并用更新的关键帧位姿来更新地图点位置
                        for (size_t i = 0; i < vpMPs.size(); i++)
                        {
                            ORB_SLAM2::MapPoint *pMP = vpMPs[i];

                            if (pMP->isBad())
                                continue;

                            // 如果这个地图点直接参与到了全局BA优化的过程,那么就直接重新设置器位姿即可
                            if (pMP->mnBAGlobalForKF == nLoopKF)
                            {
                                // If optimized by Global BA, just update
                                pMP->SetWorldPos(pMP->mPosGBA);
                            }
                            else
                            {
                                // 如这个地图点并没有直接参与到全局BA优化的过程中,那么就使用其参考关键帧的新位姿来优化自己的坐标
                                // Update according to the correction of its reference keyframe
                                ORB_SLAM2::KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();

                                // 如果参考关键帧并没有经过此次全局BA优化，就跳过
                                if (pRefKF->mnBAGlobalForKF != nLoopKF)
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
                        thread_recorder->recordSubprocessStop("MapUpdate");

                        // 释放
                        mpLocalMapper->Release();

                        cout << "Map updated!" << endl;
                    }

                    mbFinishedGBA = true;
                    mbRunningGBA = false;
                }
            }
        };
    } // ORB_SLAM2
} // SLAM_Benchmark