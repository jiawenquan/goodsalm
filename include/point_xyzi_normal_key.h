#pragma once

#include <pcl/point_types.h>

namespace goodsalm {

struct PointXYZINormalKey
{
    PCL_ADD_POINT4D;                  // add x,y,z fields
    float intensity;                  // add intensity field
    PCL_ADD_NORMAL4D;                 // add normal_x, normal_y, normal_z fields
    int key;                          // add key field
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // ensure proper alignment
} EIGEN_ALIGN16;                      // enforce SSE boundary alignment

} // namespace my_namespace

// Register the new point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(goodsalm::PointXYZINormalKey,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, normal_x, normal_x)
                                  (float, normal_y, normal_y)
                                  (float, normal_z, normal_z)
                                  (int, key, key))
