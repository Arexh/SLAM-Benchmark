#pragma once

#include "ORB_SLAM2_CUBA/include/LocalMapping.h"
#include "ORB_SLAM2_CUBA/include/Optimizer.h"

#include "ThreadRecorder.h"
#include "SystemRecorder.h"

namespace SLAM_Benchmark
{
namespace ORB_SLAM2_CUBA_Inject
{
    class LocalMapping : public ORB_SLAM2_CUBA::LocalMapping
    {
    public:
        LocalMapping(ORB_SLAM2_CUBA::Map* pMap, const float bMonocular) : ORB_SLAM2_CUBA::LocalMapping(pMap, bMonocular) {}

        void Run() override
        {
            SLAM_Benchmark::ThreadRecorder *thread_recorder = SLAM_Benchmark::SystemRecorder::getInstance(SLAM_Benchmark::SystemName::ORB_SLAM2_CUBA)->getThreadRecorder("LocalMapping");

            thread_recorder->recordThreadCreate();

            // 标记状态，表示当前run函数正在运行，尚未结束
            mbFinished = false;
            // 主循环
            while (1)
            {
                // Tracking will see that Local Mapping is busy
                // Step 1 告诉Tracking，LocalMapping正处于繁忙状态，请不要给我发送关键帧打扰我
                // LocalMapping线程处理的关键帧都是Tracking线程发来的
                SetAcceptKeyFrames(false);

                // Check if there are keyframes in the queue
                // 等待处理的关键帧列表不为空
                if (CheckNewKeyFrames())
                {
                    thread_recorder->recordThreadProcessStart();

                    thread_recorder->recordSubprocessStart("ProcessNewKeyFrame");
                    // BoW conversion and insertion in Map
                    // Step 2 处理列表中的关键帧，包括计算BoW、更新观测、描述子、共视图，插入到地图等
                    ProcessNewKeyFrame();
                    thread_recorder->recordSubprocessStop("ProcessNewKeyFrame");


                    thread_recorder->recordSubprocessStart("MapPointCulling");
                    // Check recent MapPoints
                    // Step 3 根据地图点的观测情况剔除质量不好的地图点
                    MapPointCulling();
                    thread_recorder->recordSubprocessStop("MapPointCulling");

                    thread_recorder->recordSubprocessStart("CreateNewMapPoints");
                    // Triangulate new MapPoints
                    // Step 4 当前关键帧与相邻关键帧通过三角化产生新的地图点，使得跟踪更稳
                    CreateNewMapPoints();
                    thread_recorder->recordSubprocessStop("CreateNewMapPoints");

                    // 已经处理完队列中的最后的一个关键帧
                    if (!CheckNewKeyFrames())
                    {
                        thread_recorder->recordSubprocessStart("SearchInNeighbors");
                        // Find more matches in neighbor keyframes and fuse point duplications
                        //  Step 5 检查并融合当前关键帧与相邻关键帧帧（两级相邻）中重复的地图点
                        SearchInNeighbors();
                        thread_recorder->recordSubprocessStop("SearchInNeighbors");
                    }

                    // 终止BA的标志
                    mbAbortBA = false;

                    // 已经处理完队列中的最后的一个关键帧，并且闭环检测没有请求停止LocalMapping
                    if (!CheckNewKeyFrames() && !stopRequested())
                    {
                        // Local BA
                        // Step 6 当局部地图中的关键帧大于2个的时候进行局部地图的BA
                        if (mpMap->KeyFramesInMap() > 2) // 注意这里的第二个参数是按地址传递的,当这里的 mbAbortBA 状态发生变化时，能够及时执行/停止BA
                        {
                            thread_recorder->recordSubprocessStart("LocalBundleAdjustment");
                            ORB_SLAM2_CUBA::Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpMap);
                            thread_recorder->recordSubprocessStop("LocalBundleAdjustment");
                        }

                        thread_recorder->recordSubprocessStart("KeyFrameCulling");
                        // Check redundant local Keyframes
                        // Step 7 检测并剔除当前帧相邻的关键帧中冗余的关键帧
                        // 冗余的判定：该关键帧的90%的地图点可以被其它关键帧观测到
                        KeyFrameCulling();
                        thread_recorder->recordSubprocessStop("KeyFrameCulling");
                    }

                    // Step 8 将当前帧加入到闭环检测队列中
                    // 注意这里的关键帧被设置成为了bad的情况,这个需要注意
                    mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);

                    thread_recorder->recordThreadProcessStop();
                }
                else if (Stop()) // 当要终止当前线程的时候
                {
                    // Safe area to stop
                    while (isStopped() && !CheckFinish())
                    {
                        // 如果还没有结束利索,那么等
                        // usleep(3000);
                        std::this_thread::sleep_for(std::chrono::milliseconds(3));
                    }
                    // 然后确定终止了就跳出这个线程的主循环
                    if (CheckFinish())
                        break;
                }

                // 查看是否有复位线程的请求
                ResetIfRequested();

                // Tracking will see that Local Mapping is not busy
                SetAcceptKeyFrames(true);

                // 如果当前线程已经结束了就跳出主循环
                if (CheckFinish())
                    break;

                //usleep(3000);
                std::this_thread::sleep_for(std::chrono::milliseconds(3));
            }

            // 设置线程已经终止
            SetFinish();

            thread_recorder->recordThreadDestory();
        }
    };
} // ORB_SLAM2
} // SLAM_Benchmark