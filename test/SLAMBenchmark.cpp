#include <iostream>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sys/wait.h>

#include "BenchmarkManager.h"

using namespace std;
using namespace SLAM_Benchmark;

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        cerr << "Usage: path_to_config" << endl;
        return 1;
    }
    string command(argv[0]);
    string path_prefix = command.substr(0, command.find_last_of("/"));

    std::ifstream config_file(argv[1], std::ifstream::binary);
    nlohmann::ordered_json config = nlohmann::ordered_json::parse(config_file);
    for (int i = 0; i < config["Tasks"].size(); i++) {
        nlohmann::json task = config["Tasks"][i];
        cout << "----------------------- Task " << i << " Start -----------------------" << endl;

        pid_t parent = getpid();
        pid_t pid = fork();
        if (pid == -1)
        {
            // error, failed to fork()
        } 
        else if (pid > 0)
        {
            int status;
            waitpid(pid, &status, 0);
        }
        else 
        {
            // we are the child
            string s = path_prefix + "/SingleBenchmark";
            cout << s << endl;
            execl(s.c_str(), s.c_str(), string(task["SystemName"]).c_str(), string(task["Dataset"]).c_str(), string(task["DatasetPath"]).c_str(), NULL);
            exit(0);
        }

        cout << "------------------------ Task " << i << " End ------------------------" << endl << endl;
    }

    return 0;
}
