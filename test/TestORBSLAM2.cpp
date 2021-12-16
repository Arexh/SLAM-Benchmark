/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include <unistd.h>

#include <System.h>
#include "DatasetLoader.h"
#include "BenchmarkManager.h"

using namespace std;
using namespace SLAM_Benchmark;

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        cerr << endl << "Usage: path_to_sequence" << endl;
        return 1;
    }

    BenchmarkManager::benchmark(SystemName::ORB_SLAM2, DatasetName::TUM, argv[1]);

    return 0;
}
