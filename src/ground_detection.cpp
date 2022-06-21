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
// namespace pcl
// {
void RemoveNaNFromPointCloud(pcl::PointCloud<pcl::XYZIL> &cloud_in, pcl::PointCloud<pcl::XYZIL> &cloud_out)
{
  // If the clouds are not the same, prepare the output
  if (&cloud_in != &cloud_out)
  {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize(cloud_in.points.size());
  }
  // Reserve enough space for the indices
  // index.resize(cloud_in.points.size());
  size_t j = 0;

  // If the data is dense, we don't need to check for NaN
  // if (cloud_in.is_dense) // 判断点云中是否是 dense 点云
  // {
  //   // Simply copy the data											// 如果是 dense 点云，则输出点云 = 输入点云
  //   cloud_out = cloud_in;
  //   // for (j = 0; j < cloud_out.points.size(); ++j)
  //   // index[j] = static_cast<int>(j);
  // }
  // else
  {
    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
      if (!pcl::isFinite(cloud_in.points[i]) //返回一个布尔值，判断当前点的值是不是正常数值
      )
        continue;
      cloud_out.points[j] = cloud_in.points[i];
      // index[j] = static_cast<int>(i);
      j++;
    }
    if (j != cloud_in.points.size())
    {
      // Resize to the correct size
      cloud_out.points.resize(j);
      // index.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);

    // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
    cloud_out.is_dense = true; // 将去除NaN后的点云设置为dense点云
  }
}
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
  cout << cfg.savepath + filename << endl;

  // std::vector<int> indices;
  pcl::PointCloud<pcl::XYZIL>::Ptr out(new pcl::PointCloud<pcl::XYZIL>);
  RemoveNaNFromPointCloud(*cloud_with_label, *out);
  // writer.writeBinary<pcl::XYZIL>(cfg.savepath + filename, *cloud_with_label);
  writer.write<pcl::XYZIL>(cfg.savepath + filename, *cloud_with_label, false);
  // writer.writeASCII<pcl::XYZIL>(cfg.savepath + filename, *cloud_with_label, false);
}
