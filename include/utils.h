#ifndef UTILS_H
#define UTILS_H

#include "common_headers.h"

namespace rgbd_tutor
{

struct CAMERA_INTRINSIC_PARAMETERS
{
    // 标准内参
    double cx=0, cy=0, fx=0, fy=0, scale=0;
    // 畸变因子
    double d0=0, d1=0, d2=0, d3=0, d4=0;
};

inline double norm_translate( const Eigen::Isometry3d& T )
{
    return sqrt( T(0,3)*T(0,3) + T(1,3)*T(1,3) + T(2,3)*T(2,3) );
}

inline double norm_rotate( const Eigen::Isometry3d& T )
{
    return acos( 0.5*(T(0,0)+T(1,1)+T(2,2) - 1) );
}


}

#endif // UTILS_H
