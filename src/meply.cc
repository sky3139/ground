//点的类型的头文件
#include <pcl/point_types.h>
//点云文件IO（pcd文件和ply文件）
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
// kd树
#include <pcl/kdtree/kdtree_flann.h>
//特征提取
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
//重构
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
//可视化
#include <pcl/visualization/pcl_visualizer.h>
//多线程
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
int main()
{
    pcl::PolygonMesh mesh1;
    pcl::io::loadPolygonFile("/home/u20/文档/convhull_3d-master/mply/51.ply", mesh1);
    pcl::PolygonMesh mesh2;
    pcl::io::loadPolygonFile("/home/u20/文档/convhull_3d-master/mply/88.ply", mesh2);
    pcl::PolygonMesh mesh3;
    bool ret = mesh3.concatenate(mesh1, mesh2);

	pcl::visualization::PCLVisualizer viewer("asd"); 
        viewer.addPolygonMesh( mesh1,"asd",1);

    // 开始显示2种方法,任选其一
    // 1. 阻塞式
    viewer.spin();

    if (ret)
        pcl::io::savePLYFile("mesh3D.ply", mesh3);
    else
    {
        assert(0);
    }
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPLYFile<pcl::PointXYZ>("C:\\Users\\A\\Desktop\\c\\A10-16008\\mesh3D-1.ply", *cloud1);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPLYFile<pcl::PointX
    // YZ>("C:\\Users\\A\\Desktop\\c\\A10-16008\\mesh3D-2.ply", *cloud2);
    // int sum = cloud1->points.size();
    // cout << "point_1_number:" << sum << endl;
    // rename("C:\\Users\\A\\Desktop\\c\\A10-16008\\mesh3D-1.ply", "C:\\Users\\A\\Desktop\\c\\A10-16008\\mesh3D-1.txt");

    // fstream fin("C:\\Users\\A\\Desktop\\c\\A10-16008\\mesh3D-1.txt", ios::in);
    // if (!fin)
    // {
    //     cerr << "can not open file" << endl;
    //     return -1;
    // }

    // char c1;
    // int lineCnt = 0;
    // while (fin.get(c1))
    // {
    //     if (c1 == '\n')
    //         lineCnt++;
    // }
    // cout << lineCnt + 1 + cloud2->points.size() << endl;
    // int total = lineCnt + 1 + cloud2->points.size();
    // fin.close();
    // rename("C:\\Users\\A\\Desktop\\c\\A10-16008\\mesh3D-1.txt", "C:\\Users\\A\\Desktop\\c\\A10-16008\\mesh3D-1.ply");
    // rename("C:\\Users\\A\\Desktop\\c\\A10-16008\\mesh3D.ply", "C:\\Users\\A\\Desktop\\c\\A10-16008\\mesh3D.txt");
    // fstream fin1("C:\\Users\\A\\Desktop\\c\\A10-16008\\mesh3D.txt", ios::in);
    // if (!fin1)
    // {
    //     cerr << "can not open file" << endl;
    //     return -1;
    // }
    // char cc;
    // lineCnt = 0;
    // while (fin1.get(cc))
    // {
    //     if (cc == '\n')
    //         lineCnt++;
    // }
    // cout << lineCnt + 1 << endl;
    // int txttotal = lineCnt + 1;
    // fin1.close();
    // std::queue<int> a;
    // std::queue<int> b;
    // std::queue<int> c;
    // std::queue<int> d;

    // ifstream f;//读权限变量 f
    // std::queue<std::string> r;//用于存储
    // f.open("C:\\Users\\A\\Desktop\\c\\A10-16008\\mesh3D.txt");//文件text需要存放在当前cpp文件的根目录中
    // std::string linee, s;
    // int txtLine = 1;
    // while (getline(f, linee))        //从文件中读取一行存放在line中
    // {
    //     int  aaa, bbb, ccc, ddd;
    //     r.push(linee);
    //     if (txtLine >= total && txtLine < txttotal)
    //     {
    //         std::istringstream is(linee);
    //         is >> aaa >> bbb >> ccc >> ddd;
    //         cout << aaa << " " << bbb << " " << ccc << "" << ddd << endl;
    //         a.push(aaa);
    //         b.push(bbb);
    //         c.push(ccc);
    //         d.push(ddd);
    //     }
    //     txtLine++;
    // }
    // ifstream in;
    // in.open("C:\\Users\\A\\Desktop\\c\\A10-16008\\mesh3D.txt");
    // std::string strFileData = "";
    // int line = 1;
    // char tmpLineData[1024] = { 0 };
    // while (in.getline(tmpLineData, sizeof(tmpLineData)))
    // {
    //     if (line >= total && line < txttotal)
    //     {
    //         strFileData += std::string("3 " + std::to_string(b.front() - cloud2->points.size()) + " " + std::to_string(c.front() - cloud2->points.size()) + " " + std::to_string(d.front() - cloud2->points.size()));
    //         strFileData += "\n";
    //         b.pop();
    //         c.pop();
    //         d.pop();
    //     }
    //     else
    //     {
    //         strFileData += std::string(tmpLineData);
    //         strFileData += "\n";
    //     }
    //     line++;
    // }
    // in.close();
    // //写入文件
    // ofstream out;
    // out.open("C:\\Users\\A\\Desktop\\c\\A10-16008\\mesh3D.txt");
    // out.flush();
    // out << strFileData;
    // out.close();
}