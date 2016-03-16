/*************************************************************************
	> File Name: orb.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年02月29日 星期一 12时14分06秒
 ************************************************************************/

#include <iostream>
#include "common_headers.h"
#include "converter.h"
#include "orb.h"
using namespace std;

using namespace rgbd_tutor;

vector<cv::DMatch> OrbFeature::match( const RGBDFrame::Ptr& frame1, const RGBDFrame::Ptr& frame2 ) const
{
    vector< vector<cv::DMatch> > matches_knn;
    cv::Mat desp1 = frame1->getAllDescriptors();
    cv::Mat desp2 = frame2->getAllDescriptors();
    matcher->knnMatch( desp1, desp2, matches_knn, 2 );
    vector< cv::DMatch > matches;
    for ( size_t i=0; i<matches_knn.size(); i++ )
    {
        if (matches_knn[i][0].distance < knn_match_ratio * matches_knn[i][1].distance )
        matches.push_back( matches_knn[i][0] );
    }
    return matches;
}
