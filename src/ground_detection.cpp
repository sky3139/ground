// pcl progressive morphological filter--
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include "ground_detection.h"
#include "myPointType.h"
#include "apmf.h"
#include <pcl/filters/filter.h>
#define PointXYZIL PointXYZI

#include <fstream>
template<typename POINT>
void pcdwrite(std::string file_name, pcl::PointCloud<POINT> &cloud, std::vector<int> &index)
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
  int cnt = 0;
  for (auto &it : cloud.points)
  {
    fprintf(fp, "%f %f %f %d %d\n", it.x, it.y, it.z, (int)it.intensity, index[cnt]);
    cnt++;
  }
  fclose(fp);
}

#include "pcd_io.h"
void ground_detection(std::string path, std::string filename, const config &cfg)
{
  pcl::PointCloud<pcl::PointXYZIL>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZIL>);
  std::vector<int> indices;
  // 读取点云
  pcl::MPCDReader reader;
  {
    // mtime mt("reader");
    reader.read<pcl::PointXYZIL>(path + filename, *cloud_src);
  }
  {
    // mtime mt("ex pl");
    pcl::Apmf<pcl::PointXYZIL> pmf;
    // pcl::ApproximateProgressiveMorphologicalFilter<pcl::PointXYZIL> pmf;
    // pcl::ProgressiveMorphologicalFilter<pcl::PointXYZIL> pmf;
    pmf.setNumberOfThreads(12);
    pmf.setInputCloud(cloud_src);
    pmf.setMaxWindowSize(20);
    pmf.setSlope(1.0f);
    pmf.setInitialDistance(0.3f); // 0.5
    pmf.setMaxDistance(0.6f);     // 3
    pmf.max_height_ = cfg.max_height_;
    pmf.extract(indices);
  }
  // cout << cfg.savepath + filename << endl;
  pcdwrite(cfg.savepath + filename, *cloud_src, indices);
}
