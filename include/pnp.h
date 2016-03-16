#ifndef PNP_H
#define PNP_H

#include "common_headers.h"
#include "rgbdframe.h"
#include "parameter_reader.h"
#include "orb.h"

/*
 * 求解pnp问题
 * 使用g2o作为优化方法求解
 */

namespace rgbd_tutor
{
struct PNP_INFORMATION
{
    // 记录pnp过程中的信息
    int numFeatureMatches   =0;         // 匹配到的特征
    int numInliers          =0;         // pnp内点
    Eigen::Isometry3d   T   = Eigen::Isometry3d::Identity(); //相对变换

};

class PnPSolver
{
public:
    PnPSolver( const rgbd_tutor::ParameterReader& para, const rgbd_tutor::OrbFeature& orbFeature ):
        parameterReader( para ),
        orb( orbFeature )
    {
        min_inliers = para.getData<int>("pnp_min_inliers");
        min_match = para.getData<int>("pnp_min_matches");
    }

    //  求解pnp问题
    //  输入  2d点，3d点，相机内参
    //  输出  变换矩阵T（可设置初值），inliers index
    //  返回  是否成功
    bool    solvePnP( const vector<cv::Point2f>& img, const vector<cv::Point3f>& obj, const rgbd_tutor::CAMERA_INTRINSIC_PARAMETERS& camera, vector<int>& inliersIndex,
                      Eigen::Isometry3d& transform );

    //  更懒的求解方式：直接给定两个帧，匹配由内部计算
    //  输入  两个帧
    //  输出  变换矩阵和inlier index
    bool    solvePnPLazy( const rgbd_tutor::RGBDFrame::Ptr & frame1, const rgbd_tutor::RGBDFrame::Ptr frame2, PNP_INFORMATION& pnp_information, bool drawMatches=false );

protected:
    const   rgbd_tutor::ParameterReader& parameterReader;
    const   rgbd_tutor::OrbFeature& orb;

    // 参数
    int     min_inliers =10;
    int     min_match   =30;
};

}

#endif // PNP_H
