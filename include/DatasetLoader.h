#pragma once

#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

namespace SLAM_Benchmark
{
    class DatasetLoader
    {
    protected:
        std::string m_dataset_path;
        std::vector<std::string> m_image_file_name;
        std::vector<double> m_timestamp;

    public:
        DatasetLoader(const std::string dataset_path) : m_dataset_path(dataset_path) {}

        virtual void init() = 0;

        std::vector<double> getTimestamp() const { return m_timestamp; }

        int getSize() { return m_timestamp.size(); }

        virtual bool loadImage(const int index, cv::Mat &image) = 0;
    };

    class TUMDatasetLoader : public DatasetLoader
    {
    private:
        const std::string m_index_file_path = "rgb.txt";
    public:
        TUMDatasetLoader(const std::string dataset_path) : DatasetLoader(dataset_path) {
            cout << "Loading TUM dataset..." << endl;
            init();
            cout << "Loading TUM dataset... - done" << endl;
        }
        // copy from: https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/Examples/Monocular/mono_tum.cc?_pjax=%23js-repo-pjax-container%2C%20div%5Bitemtype%3D%22http%3A%2F%2Fschema.org%2FSoftwareSourceCode%22%5D%20main%2C%20%5Bdata-pjax-container%5D#L128-L155
        void init()
        {
            std::ifstream f;
            f.open((m_dataset_path + '/' + m_index_file_path).c_str());

            // skip first three lines
            std::string s0;
            getline(f, s0);
            getline(f, s0);
            getline(f, s0);

            while (!f.eof())
            {
                std::string s;
                getline(f, s);
                if (!s.empty())
                {
                    std::stringstream ss;
                    ss << s;
                    double t;
                    std::string sRGB;
                    ss >> t;
                    m_timestamp.push_back(t);
                    ss >> sRGB;
                    m_image_file_name.push_back(sRGB);
                }
            }
        }

        bool loadImage(const int index, cv::Mat &image)
        {
            image = cv::imread(m_dataset_path + "/" + m_image_file_name[index], cv::IMREAD_UNCHANGED);
            if(image.empty())
            {
                std::cerr << std::endl << "Failed to load image at: "
                    << m_dataset_path << "/" << m_image_file_name[index] << std::endl;
                return false;
            }
            return true;
        }
    };
}