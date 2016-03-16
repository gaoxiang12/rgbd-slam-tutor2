#ifndef FEATURE_H
#define FEATURE_H

#include "common_headers.h"

namespace rgbd_tutor
{

class Feature
{
public:
    Feature( ) {}
public:
    cv::KeyPoint    keypoint;
    cv::Mat         descriptor;
    cv::Point3f     position;       // position in 3D space
    float           observe_frequency = 0.0;      //被观测到的频率
};

}

#endif // FEATURE_H
