#include "VINS-Course/include/System.h"
#include <opencv2/opencv.hpp>

#include "ThreadRecorder.h"
#include "SystemRecorder.h"

namespace SLAM_Benchmark
{
    namespace VINS_Course_Inject
    {
        class VINSSystem : public System
        {
            const int nDelayTimes = 1;
            std::string m_project_dir;
            std::string m_dataset_path;

        public:
            VINSSystem(std::string project_dir, std::string dataset_path) : System(project_dir + "third_party/VINS-Course/config/")
            {
                m_project_dir = project_dir + "/third_party/VINS-Course";
                m_dataset_path = dataset_path;
            }

            // thread: visual-inertial odometry
            void ProcessBackEnd() override
            {
                SLAM_Benchmark::ThreadRecorder *publish_record = SLAM_Benchmark::SystemRecorder::getInstance(SLAM_Benchmark::SystemName::VINS_Course)->getPublishRecord();

                cout << "1 ProcessBackEnd start" << endl;
                while (bStart_backend)
                {
                    // cout << "1 process()" << endl;
                    vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements;

                    unique_lock<mutex> lk(m_buf);
                    con.wait(lk, [&]
                             { return (measurements = getMeasurements()).size() != 0; });
                    if (measurements.size() > 1)
                    {
                        cout << "1 getMeasurements size: " << measurements.size()
                             << " imu sizes: " << measurements[0].first.size()
                             << " feature_buf size: " << feature_buf.size()
                             << " imu_buf size: " << imu_buf.size() << endl;
                    }
                    lk.unlock();
                    m_estimator.lock();
                    publish_record->recordThreadProcessStart();
                    for (auto &measurement : measurements)
                    {
                        auto img_msg = measurement.second;
                        double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
                        for (auto &imu_msg : measurement.first)
                        {
                            double t = imu_msg->header;
                            double img_t = img_msg->header + estimator.td;
                            if (t <= img_t)
                            {
                                if (current_time < 0)
                                    current_time = t;
                                double dt = t - current_time;
                                assert(dt >= 0);
                                current_time = t;
                                dx = imu_msg->linear_acceleration.x();
                                dy = imu_msg->linear_acceleration.y();
                                dz = imu_msg->linear_acceleration.z();
                                rx = imu_msg->angular_velocity.x();
                                ry = imu_msg->angular_velocity.y();
                                rz = imu_msg->angular_velocity.z();
                                estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                                // printf("1 BackEnd imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                            }
                            else
                            {
                                double dt_1 = img_t - current_time;
                                double dt_2 = t - img_t;
                                current_time = img_t;
                                assert(dt_1 >= 0);
                                assert(dt_2 >= 0);
                                assert(dt_1 + dt_2 > 0);
                                double w1 = dt_2 / (dt_1 + dt_2);
                                double w2 = dt_1 / (dt_1 + dt_2);
                                dx = w1 * dx + w2 * imu_msg->linear_acceleration.x();
                                dy = w1 * dy + w2 * imu_msg->linear_acceleration.y();
                                dz = w1 * dz + w2 * imu_msg->linear_acceleration.z();
                                rx = w1 * rx + w2 * imu_msg->angular_velocity.x();
                                ry = w1 * ry + w2 * imu_msg->angular_velocity.y();
                                rz = w1 * rz + w2 * imu_msg->angular_velocity.z();
                                estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                                //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                            }
                        }

                        // cout << "processing vision data with stamp:" << img_msg->header
                        //     << " img_msg->points.size: "<< img_msg->points.size() << endl;

                        // TicToc t_s;
                        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
                        for (unsigned int i = 0; i < img_msg->points.size(); i++)
                        {
                            int v = img_msg->id_of_point[i] + 0.5;
                            int feature_id = v / NUM_OF_CAM;
                            int camera_id = v % NUM_OF_CAM;
                            double x = img_msg->points[i].x();
                            double y = img_msg->points[i].y();
                            double z = img_msg->points[i].z();
                            double p_u = img_msg->u_of_point[i];
                            double p_v = img_msg->v_of_point[i];
                            double velocity_x = img_msg->velocity_x_of_point[i];
                            double velocity_y = img_msg->velocity_y_of_point[i];
                            assert(z == 1);
                            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                            image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
                        }
                        TicToc t_processImage;
                        estimator.processImage(image, img_msg->header);

                        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
                        {
                            Vector3d p_wi;
                            Quaterniond q_wi;
                            q_wi = Quaterniond(estimator.Rs[WINDOW_SIZE]);
                            p_wi = estimator.Ps[WINDOW_SIZE];
                            vPath_to_draw.push_back(p_wi);
                            double dStamp = estimator.Headers[WINDOW_SIZE];
                            cout << "1 BackEnd processImage dt: " << fixed << t_processImage.toc() << " stamp: " << dStamp << " p_wi: " << p_wi.transpose() << endl;
                            ofs_pose << fixed << dStamp << " " << p_wi(0) << " " << p_wi(1) << " " << p_wi(2) << " "
                                     << q_wi.w() << " " << q_wi.x() << " " << q_wi.y() << " " << q_wi.z() << endl;
                        }
                    }
                    publish_record->recordThreadProcessStop();
                    m_estimator.unlock();
                }
            }

            void PubImuDataCopy()
            {
                string sImu_data_file = m_project_dir + "/config/MH_05_imu0.txt";
                cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
                ifstream fsImu;
                fsImu.open(sImu_data_file.c_str());
                if (!fsImu.is_open())
                {
                    cerr << "Failed to open imu file! " << sImu_data_file << endl;
                    return;
                }

                std::string sImu_line;
                double dStampNSec = 0.0;
                Vector3d vAcc;
                Vector3d vGyr;
                while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
                {
                    std::istringstream ssImuData(sImu_line);
                    ssImuData >> dStampNSec >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
                    // cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
                    PubImuData(dStampNSec / 1e9, vGyr, vAcc);
                    usleep(5000 * nDelayTimes);
                }
                fsImu.close();
            }

            void PubImageDataCopy()
            {
                string sImage_file = m_project_dir + "/config/MH_05_cam0.txt";

                cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

                ifstream fsImage;
                fsImage.open(sImage_file.c_str());
                if (!fsImage.is_open())
                {
                    cerr << "Failed to open image file! " << sImage_file << endl;
                    return;
                }

                std::string sImage_line;
                double dStampNSec;
                string sImgFileName;

                // cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
                while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
                {
                    std::istringstream ssImuData(sImage_line);
                    ssImuData >> dStampNSec >> sImgFileName;
                    // cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;
                    string imagePath = m_dataset_path + "/cam0/data/" + sImgFileName;

                    cv::Mat img = cv::imread(imagePath.c_str(), 0);
                    if (img.empty())
                    {
                        cerr << "image is empty! path: " << imagePath << endl;
                        return;
                    }
                    PubImageData(dStampNSec / 1e9, img);
                    // cv::imshow("SOURCE IMAGE", img);
                    // cv::waitKey(0);
                    usleep(50000 * nDelayTimes);
                }
                fsImage.close();
            }
        };
    }
}