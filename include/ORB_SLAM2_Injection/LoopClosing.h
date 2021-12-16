#pragma once

#include "ORB_SLAM2_detailed_comments/include/LoopClosing.h"

#include "ThreadRecorder.h"

namespace SLAM_Benchmark
{
namespace ORB_SLAM2_Inject
{
    class LoopClosing : public ORB_SLAM2::LoopClosing
    {
    public:
        LoopClosing(ORB_SLAM2::Map* pMap, ORB_SLAM2::KeyFrameDatabase* pDB, ORB_SLAM2::ORBVocabulary* pVoc, const bool bFixScale) : ORB_SLAM2::LoopClosing(pMap, pDB, pVoc, bFixScale) {}
        // 回环线程主函数
        void Run()
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
                    // Detect loop candidates and check covisibility consistency
                    if (DetectLoop())
                    {
                        // Compute similarity transformation [sR|t]
                        // In the stereo/RGBD case s=1
                        if (ComputeSim3())
                        {
                            // Perform loop fusion and pose graph optimization
                            CorrectLoop();
                        }
                    }
                }

                // 查看是否有外部线程请求复位当前线程
                ResetIfRequested();

                // 查看外部线程是否有终止当前线程的请求,如果有的话就跳出这个线程的主函数的主循环
                if (CheckFinish())
                    break;

                //usleep(5000);
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }

            // 运行到这里说明有外部线程请求终止当前线程,在这个函数中执行终止当前线程的一些操作
            SetFinish();
        }
    };
} // ORB_SLAM2
} // SLAM_Benchmark