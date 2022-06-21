#pragma once
#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <pcl/point_types.h>
#include "pcd_io.h"
struct config
{
    bool isamp;
    std::string temp, inPath, savepath;
    float max_height_;
    int thread_num;
    int fun_number;
};

void ground_detection(std::string path, std::string filename, const config &cfg);
class mtime
{
public:
    std::chrono::high_resolution_clock::time_point start_time_;
    std::string info = "";
    mtime(std::string _info)
    {
        info = _info;
        start_time_ = std::chrono::high_resolution_clock::now();
    }
    ~mtime()
    {
        float d2 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time_).count();
        printf("%-10s:  %6.2f ms\n", info.c_str(), d2 * 0.001f);
    }
};
template <typename POINT>
void pcdwrite(std::string file_name, pcl::PointCloud<POINT> &cloud, std::vector<int> &index);
