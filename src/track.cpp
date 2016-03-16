#include "track.h"
#include <unistd.h>

// tracker的主线程
using namespace rgbd_tutor;

Eigen::Isometry3d    Tracker::updateFrame( RGBDFrame::Ptr& newFrame )
{
    unique_lock<mutex> lck(adjustMutex);
    currentFrame = newFrame;
    if ( state == NOT_READY )
    {
        initFirstFrame( );
        return Eigen::Isometry3d::Identity();
    }
    if ( state == OK )
    {
        trackRefFrame();
        return currentFrame->getTransform();
    }
    // state = LOST

    lostRecover();
    return currentFrame->getTransform();

}

void    Tracker::initFirstFrame( )
{
    orb->detectFeatures( currentFrame );
    refFrames.push_back(currentFrame);
    speed = Eigen::Isometry3d::Identity();
    state = OK;
}

void    Tracker::trackRefFrame()
{
    //adjustMutex.lock();
    // 初始值
    currentFrame->setTransform( speed * refFrames.back()->getTransform() );
    orb->detectFeatures( currentFrame );
    
    // build local BA
    vector<cv::Point3f> obj;
    vector<cv::Point2f> img;
    for (auto pFrame: refFrames)
    {
        vector<cv::DMatch> matches = orb->match( pFrame, currentFrame );
        vector<cv::DMatch>  validMatches;
        Eigen::Isometry3d invPose = pFrame->getTransform().inverse();
        for (auto m:matches)
        {
            cv::Point3f pObj = pFrame->features[m.queryIdx].position;
            if (pObj == cv::Point3f(0,0,0))
                continue;
            Eigen::Vector4d vec = invPose * Eigen::Vector4d(pObj.x, pObj.y, pObj.z,1 );
            obj.push_back( cv::Point3f(vec(0), vec(1), vec(2) ) );
            img.push_back( currentFrame->features[m.trainIdx].keypoint.pt );
        }
    }
    
    if ( img.size() < 15 )
    {
        cntLost ++;
        if (cntLost > max_lost_frame)
        {
            state = LOST;
        }
        return;
    }
    
    vector<int> inlierIndex;
    Eigen::Isometry3d T = speed * lastPose;
    bool b = pnp->solvePnP( img, obj, currentFrame->camera, inlierIndex, T );
    if ( inlierIndex.size() < 15 )
    {
        cntLost ++;
        if (cntLost > max_lost_frame)
        {
            state = LOST;
        }
        return;
    }
    
    currentFrame->setTransform( T );
    cntLost = 0;
    speed = T * lastPose.inverse();
    lastPose = currentFrame->getTransform();
    refFrames.push_back( currentFrame );
    while (refFrames.size() > refFramesSize )
    {
        refFrames.pop_front();
    }
    
    //cout<<"speed="<<endl<<speed.matrix()<<endl;
}

void    Tracker::lostRecover()
{
    cout<<"trying to recover from lost"<<endl;
    orb->detectFeatures( currentFrame );
    currentFrame->setTransform( refFrames.back()->getTransform() );
    refFrames.clear();
    refFrames.push_back( currentFrame );
    state = OK;
    cntLost = 0;
    cout<<"recover returned"<<endl;
}
