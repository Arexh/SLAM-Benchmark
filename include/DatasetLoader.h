#pragma once

#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include<iomanip>

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

    class EuRoCDatasetLoader : public DatasetLoader
    {
    private:
        std::string m_timestamps_path;
        std::string m_sequence;
    public:
        EuRoCDatasetLoader(const std::string dataset_path, const std::string timestamps_path, const::string sequence) : DatasetLoader(dataset_path) {
            m_timestamps_path = timestamps_path;
            m_sequence = sequence;
            cout << m_timestamps_path << endl;
            cout << "Loading EuRoc dataset..." << endl;
            init();
            cout << "Loading EuRoc dataset... - done" << endl;
        }

        void init() {
            ifstream fTimes;
            fTimes.open((m_timestamps_path + "/" + m_sequence + ".txt").c_str());
            m_timestamp.reserve(5000);
            m_image_file_name.reserve(5000);
            while(!fTimes.eof())
            {
                string s;
                getline(fTimes,s);
                if(!s.empty())
                {
                    stringstream ss;
                    ss << s;
                    m_image_file_name.push_back(ss.str() + ".png");
                    double t;
                    ss >> t;
                    m_timestamp.push_back(t/1e9);
                }
            }
        }

        bool loadImage(const int index, cv::Mat &image)
        {
            image = cv::imread(m_dataset_path + "/mav0/cam0/data/" + m_image_file_name[index], cv::IMREAD_UNCHANGED);
            if(image.empty())
            {
                std::cerr << std::endl << "Failed to load image at: "
                    << m_dataset_path << "/mav0/cam0/data/" << m_image_file_name[index] << std::endl;
                return false;
            }
            return true;
        }
    };

    class KITTIDatasetLoader : public DatasetLoader
    {
    public:
        KITTIDatasetLoader(const std::string dataset_path) : DatasetLoader(dataset_path) {
            cout << "Loading EuRoC dataset..." << endl;
            init();
            cout << "Loading EuRoC dataset... - done" << endl;
        }

        void init() {
            ifstream fTimes;
            fTimes.open((m_dataset_path + "/times.txt").c_str());
            while(!fTimes.eof())
            {
                string s;
                getline(fTimes,s);
                if(!s.empty())
                {
                    stringstream ss;
                    ss << s;
                    double t;
                    ss >> t;
                    m_timestamp.push_back(t);
                }
            }
            string strPrefixLeft = m_dataset_path + "/image_0/";

            const int nTimes = m_timestamp.size();
            m_image_file_name.resize(nTimes);

            for(int i=0; i<nTimes; i++)
            {
                stringstream ss;
                ss << std::setfill('0') << std::setw(6) << i;
                m_image_file_name[i] = strPrefixLeft + ss.str() + ".png";
            }
        }

        bool loadImage(const int index, cv::Mat &image)
        {
            image = cv::imread(m_image_file_name[index], cv::IMREAD_UNCHANGED);
            if(image.empty())
            {
                std::cerr << std::endl << "Failed to load image at: "
                    << m_image_file_name[index] << std::endl;
                return false;
            }
            return true;
        }
    };
}