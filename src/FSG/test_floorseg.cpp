
#include <chrono>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <string>

#include "FloorSegment.h"

#include "../pcd_io.h"
#include "../ground_detection.h"
void fsg_detection(std::string path, std::string filename, const config &cfg)
{
    // Read PLY FILE FOR TEST
    pcl::PointCloud<pcl::PointXYZI> cloud;

    std::string ply_file = path + filename;

    pcl::io::loadPCDFile(ply_file, cloud);

    // auto startTime = std::chrono::steady_clock::now();

    // FloorSegmentParams Initialize
    floorSegmentParams params;
    params.visualize = false; // show with true
    params.r_min_square = 0.1 * 0.1;
    params.r_max_square = 100 * 100;
    params.n_bins = 100;
    params.n_segments = 180;
    params.max_dist_to_line = 0.15;
    params.max_slope = 1;
    params.max_error_square = 0.01;
    params.long_threshold = 2.0;
    params.max_long_height = 0.1;
    params.max_start_height = 0.2;
    params.sensor_height = 1.73;
    params.line_search_angle = 0.2;
    params.n_threads = 1;

    // Run FloorSegment
    floorSegmentation segmenter(params);

    std::vector<int> labels;
    segmenter.segment(cloud, &labels);


    pcdwrite(cfg.savepath + filename, cloud, labels);

}
