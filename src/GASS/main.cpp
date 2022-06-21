

#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <chrono>

#include "GaussianFloorSegmentation.h"
template <typename POINT>
void NEWpcdwrite(std::string file_name, pcl::PointCloud<POINT> &cloud, std::vector<int> &index)
{
    FILE *fp = fopen(file_name.c_str(), "w");
    fprintf(fp, "# .PCD v0.7 - Point Cloud Data file format\n");
    fprintf(fp, "VERSION 0.7\n");
    fprintf(fp, "FIELDS x y z intensity label\n");
    fprintf(fp, "SIZE 4 4 4 4 2\n");
    fprintf(fp, "TYPE F F F F U\n");
    fprintf(fp, "COUNT 1 1 1 1 1\n");
    fprintf(fp, "WIDTH %d\n", 1800);
    fprintf(fp, "HEIGHT %d\n", 128);
    fprintf(fp, "VIEWPOINT 0 0 0 1 0 0 0\n");
    fprintf(fp, "POINTS %d\n", 230400);
    fprintf(fp, "DATA ascii\n");
    register int cnt = 0;
    std::vector<int> all(230400, 0);
    for (register int i = 0; i < index.size(); i++)
    {
        all[index[i]] = 1;
    }
    for (auto &it : cloud.points)
    {
        fprintf(fp, "%f %f %f %d %d\n", it.x, it.y, it.z, (int)it.intensity, all[cnt]);
        cnt++;
    }
    fclose(fp);
}
#include "../ground_detection.h"
void gass_detection(std::string path, std::string filename, const config &cfg)
{
    // mtime t("gass");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::MPCDReader reader;
    reader.read<pcl::PointXYZI>(path + filename, *cloud);
    pcl::GaussianFloorSegmentationParams params;

    pcl::GaussianFloorSegmentation<pcl::PointXYZI> ground_segmentation{params};
    ground_segmentation.setInputCloud(cloud);
    ground_segmentation.setKeepGround(true);
    ground_segmentation.setKeepObstacle(false);
    ground_segmentation.setKeepOverHanging(false);

    // mtime mt("indices");
    std::vector<int> &indices = ground_segmentation.getGD();
    // std::cout << cfg.savepath + filename << std::endl;
    NEWpcdwrite(cfg.savepath + filename, *cloud, indices);
}
