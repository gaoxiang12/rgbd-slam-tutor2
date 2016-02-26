#ifndef ORB_H
#define ORB_H

#include "common.h"
#include "rgbdframe.h"
#include "ORB_SLAM2/include/ORBextractor.h"

/**
 * OrbFeature
 * 对orbslam2的特征部分进行一次封装，具有提取、匹配特征的功能
 */

namespace rgbd_tutor
{

class OrbFeature
{
public:
    OrbFeature( const rgbd_tutor::ParameterReader& para )
    {
        int features  = para.getData<int>("orb_features");
        float   scale = para.getData<float>("orb_scale");
        int     level = para.getData<int>("orb_levels");
        int     ini   = para.getData<int>("orb_iniThFAST");
        int     min   = para.getData<int>("orb_minThFAST");
        extractor = make_shared<ORB_SLAM2::ORBextractor>( features, scale, level, ini, min );
        matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
        knn_match_ratio = para.getData<double>("knn_match_ratio");
    }

    // 提取特征，存放到frame的成员变量中
    void detectFeatures( rgbd_tutor::Frame& frame ) const
    {
        if (frame.rgb.channels() == 3)
        {
            // The BGR image
            cv::Mat gray;
            cv::cvtColor( frame.rgb, gray, cv::COLOR_BGR2GRAY );
            (*extractor) ( gray, cv::Mat(), frame.keypoints, frame.descriptor );
        }
        else
        {
            (*extractor) ( frame.rgb, cv::Mat(), frame.keypoints, frame.descriptor );
        }

    }

    // 匹配两个帧之间的特征描述
    vector<cv::DMatch>  match( const rgbd_tutor::Frame& frame1, const rgbd_tutor::Frame& frame2 ) const
    {
        vector< vector<cv::DMatch> > matches_knn;
        matcher->knnMatch( frame1.descriptor, frame2.descriptor, matches_knn, 2 );
        vector< cv::DMatch > matches;
        for ( size_t i=0; i<matches_knn.size(); i++ )
        {
            if (matches_knn[i][0].distance < knn_match_ratio * matches_knn[i][1].distance )
            matches.push_back( matches_knn[i][0] );
        }
        return matches;
    }

protected:
    shared_ptr<ORB_SLAM2::ORBextractor> extractor;
    cv::Ptr< cv::DescriptorMatcher > matcher;

    double knn_match_ratio =0.8;

};

}

#endif // ORB_H
