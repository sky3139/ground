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

#define PointXYZIL PointXYZI

void ground_detection(std::string path, std::string filename, const config &cfg)
{
  pcl::PointCloud<pcl::PointXYZIL>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZIL>);
  pcl::PointIndicesPtr ground(new pcl::PointIndices);
  // 读取点云
  pcl::PCDReader reader;
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
    pmf.extract(ground->indices);
  }
  pcl::PointCloud<pcl::XYZIL>::Ptr cloud_with_label(new pcl::PointCloud<pcl::XYZIL>);
  pcl::copyPointCloud(*cloud_src, *cloud_with_label);
  {
    // mtime mt("omp for");
#pragma omp for
    for (int ind : ground->indices)
    {
      cloud_with_label->points[ind].label = 1;
    }
  }
  pcl::PCDWriter writer;
  // cout<<cfg.savepath + filename<<endl;
  // writer.writeBinary<pcl::XYZIL>(cfg.savepath + filename, *cloud_with_label);
    writer.write<pcl::XYZIL>(cfg.savepath + filename, *cloud_with_label,false);
}
