#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
using namespace std;
typedef pcl::PointXYZ point;
typedef pcl::PointCloud<point> pointcloud;

int main(int argc, char **argv)
{

    // 确定文件格式
    char tmpStr[100];
    strcpy(tmpStr, argv[1]);
    char *pext = strrchr(tmpStr, '.');
    std::string extply("ply");
    std::string extpcd("pcd");
    if (pext)
    {
        *pext = '\0';
        pext++;
    }
    std::string ext(pext);
    //如果不支持文件格式，退出程序
    if (!((ext == extply) || (ext == extpcd)))
    {
        std::cout << "文件格式不支持!" << std::endl;
        std::cout << "支持文件格式：*.pcd和*.ply！" << std::endl;
        return (-1);
    }

    //根据文件格式选择输入方式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //创建点云对象指针，用于存储输入
    if (ext == extply)
    {
        if (pcl::io::loadPLYFile(argv[1], *cloud) == -1)
        {
            PCL_ERROR("Could not read ply file!\n");
            return -1;
        }
    }
    else
    {
        if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
        {
            PCL_ERROR("Could not read pcd file!\n");
            return -1;
        }
    }

    cout << "points size is:" << cloud->size() << endl;
    pcl::search::KdTree<point>::Ptr tree(new pcl::search::KdTree<point>);

    //创建存储的mls对象
    // pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::PointCloud<point> mls_points;

    //创建mls对象
    //  pcl::MovingLeastSquares<point,pcl::PointNormal> mls;

    pcl::MovingLeastSquares<point, point> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud);
    // mls.setPolynomialFit(true); //设置为true则在平滑过程中采用多项式拟合来提高精度
    mls.setPolynomialOrder(1); // MLS拟合的阶数，默认是2
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.3); //这个值越大，输出的点越多
    {
        // mls.setDilationVoxelSize(0.3);
        // mls.setDilationIterations(30);
        // mls.setUpsamplingMethod(pcl::MovingLeastSquares<point, point>::VOXEL_GRID_DILATION);
        mls.setProjectionMethod(pcl::MLSResult::ProjectionMethod::SIMPLE);
        // mls.setUpsamplingRadius(0.2);
        // mls.setUpsamplingStepSize(0.2);
        // mls.setNumberOfThreads(10);
    }
    mls.setUpsamplingMethod( pcl::MovingLeastSquares<point, point>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(0.2);
    mls.setUpsamplingStepSize(0.2);
    mls.setNumberOfThreads(10);
    mls.process(mls_points);

    cout << "mls poits size is: " << mls_points.size() << endl;

    // Save output
    pcl::io::savePLYFile("mls.ply", mls_points);
    return 0;
}
