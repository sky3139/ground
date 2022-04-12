
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include "ground_detection.h"
#include "myPointType.h"
#include <vector>
#include <numeric>
using namespace std;
std::vector<string> getfiles(string gtPath)
{
    DIR *dir;
    struct dirent *ptr;
    dir = opendir(gtPath.c_str());
    std::vector<string> files;
    //
    while ((ptr = readdir(dir)) != NULL)
    {
        if (ptr->d_name[0] == '.')
            continue;
        files.push_back(string(ptr->d_name));
    }
    closedir(dir);
    return files;
}
std::vector<float> fTPR, fFPR;
void runevo(std::string tpath, std::string lpath, std::string lbf)
{
    pcl::PointCloud<pcl::XYZIL>::Ptr cloud_groud_truth(new pcl::PointCloud<pcl::XYZIL>);
    pcl::PointCloud<pcl::XYZIL>::Ptr cloud_groud_lab(new pcl::PointCloud<pcl::XYZIL>);

    pcl::PCDReader reader;
    {
        // mtime mt("reader");
        reader.read<pcl::XYZIL>(tpath + lbf, *cloud_groud_truth);
        reader.read<pcl::XYZIL>(lpath + lbf, *cloud_groud_lab);
    }
    int TP = 0, TN = 0, FP = 0, FN = 0; //
    // ，TP：语义分割正确的地面点数量，FP：语义分割错误的地面点数量，FN：语义分割漏检的地面点数量。
    for (int i = 0; i < cloud_groud_truth->points.size(); i++)
    {
        auto tp = cloud_groud_truth->points[i];
        auto lp = cloud_groud_lab->points[i];
        switch (tp.label)
        {
        case 0: // 不是地面
            if (lp.label == 0)
            {
                TN++;
            }
            else //预测正确
            {
                FP++;
            }
            break;
        case 1: //是地面
            if (lp.label == 1)
            {
                TP++;
            }
            else
            {
                FN++;
            }
            break;
        default:
            break;
        }
    }
    fTPR.push_back((float)TP / (TP + FN));
    fFPR.push_back((float)FP / (FP + TN));
    std::cout << "TPR:" << (float)TP / (TP + FN) << " FPR:" << (float)FP / (FP + TN) << std::endl;
}
int main()
{

    config cfg;
    fstream fconfig("../evo_config.md");
    string temp, gtPath, savepath;
    int isamp;
    getline(fconfig, temp);
    getline(fconfig, gtPath);
    getline(fconfig, temp);
    fconfig >> cfg.savepath; // lable
    char ch, infile[50], outfile[50];

    std::vector<string> gtf = getfiles(gtPath);
    std::vector<string> lbf = getfiles(cfg.savepath);
    omp_set_num_threads(10);
    std::cout << gtf.size() << " " << lbf.size() << std::endl;

#pragma omp parallel for
    for (int i = 0; i < lbf.size(); i++)
    {
        // std::cout << cfg.savepath + lbf[i] << endl;
        runevo(gtPath, cfg.savepath, lbf[i]);
    }

    float sum1 = std::accumulate(std::begin(fTPR), std::end(fTPR), 0.0);
    float mean1 = sum1 / fTPR.size(); //均值

    float sum2 = std::accumulate(std::begin(fFPR), std::end(fFPR), 0.0);
    float mean2 = sum2 / fFPR.size(); //均值

    std::cout << mean1 << " " << mean2 << std::endl;

    return 0;
}