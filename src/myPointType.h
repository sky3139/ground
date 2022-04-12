#define PCL_NO_PRECOMPILE

#ifndef MYPOINTTYPE_H
#define MYPOINTTYPE_H
#include <pcl/point_types.h>
namespace pcl
{
    struct XYZIL
    {
        PCL_ADD_POINT4D;
        float intensity;
        std::uint16_t label;

        XYZIL()
        {
            label=0;
            intensity=0;
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    } EIGEN_ALIGN16;

}
POINT_CLOUD_REGISTER_POINT_STRUCT(XYZIL, // 注册点类型宏
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, label, label))

#endif