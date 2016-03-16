#ifndef MY_CONVERTER_H
#define MY_CONVERTER_H

#include "common_headers.h"
#include "rgbdframe.h"
#include "Thirdparty/orbslam_modified/include/Converter.h"

/* *
 * convert the common types in cv, eigen and g2o.
 */

namespace rgbd_tutor {
class Converter: public ORB_SLAM2::Converter
{
public:
    static  g2o::SE3Quat    toSE3Quat(const Eigen::Isometry3d& T)
    {
        Eigen::Matrix3d R = T.rotation();
        /*
        R << T(0,0) << T(0,1) << T(0,2) <<
             T(1,0) << T(1,1) << T(1,2) <<
             T(2,0) << T(2,1) << T(2,2);*/
        Eigen::Vector3d t( T(0,3), T(1,3), T(2,3) );

        return g2o::SE3Quat( R, t );
    };

    static  cv::Mat     toCvCameraMatrix( const rgbd_tutor::CAMERA_INTRINSIC_PARAMETERS& camera )
    {
        double camera_matrix_data[3][3] = {
            {camera.fx, 0, camera.cx},
            {0, camera.fy, camera.cy},
            {0, 0, 1}
        };
        return cv::Mat( 3, 3, CV_64F, camera_matrix_data );
    }

    // 将cv的平移向量和旋转矩阵转换成变换矩阵
    static  Eigen::Isometry3d   cvRT2EigenIsometry( const cv::Mat& rvec, const cv::Mat& tvec )
    {
        cv::Mat R;
        cv::Rodrigues( rvec, R );
        Eigen::Matrix3d r;
        cv::cv2eigen(R, r);


        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        Eigen::AngleAxisd angle(r);
        T = angle;
        T(0,3) = tvec.at<double>(0,0);
        T(1,3) = tvec.at<double>(0,1);
        T(2,3) = tvec.at<double>(0,2);
        return T;
    }


};

};

#endif // MY_CONVERTER_H
