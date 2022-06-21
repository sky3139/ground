#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv)
{
	// Load input file into a PointCloud<T> with an appropriate type
	//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//   pcl::PCLPointCloud2 cloud_blob;
	//   pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
	//   pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
	//* the data should be available in cloud

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

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18);		  // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3);	  // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	// 显示结果图
	pcl::visualization::PCLVisualizer *viewer=(new pcl::visualization::PCLVisualizer("3D viewer"));
	viewer->setBackgroundColor(0, 0, 0); //背景　黑色
	viewer->addPolygonMesh(triangles, "my");	 // mesh
	// viewer->addCoordinateSystem (0.10);//坐标系
	viewer->initCameraParameters(); //相机参数
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	// Finish
	return (0);
}