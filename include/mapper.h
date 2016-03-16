#ifndef MAPPER_H
#define MAPPER_H

#include "common_headers.h"
#include "rgbdframe.h"
#include "pose_graph.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

namespace rgbd_tutor
{
using namespace rgbd_tutor;

class Mapper
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    Mapper( const ParameterReader& para, PoseGraph& graph )
        : parameterReader( para ), poseGraph( graph )
    {
        resolution = para.getData<double>("mapper_resolution");
        max_distance = para.getData<double>("mapper_max_distance");

        viewerThread = make_shared<thread> ( bind( &Mapper::viewer, this ));
    }
    void shutdown()
    {
        shutdownFlag = true;
        viewerThread->join();
    }

    // viewer线程
    void viewer();

protected:
    PointCloud::Ptr generatePointCloud( const RGBDFrame::Ptr& frame );

protected:
    // viewer thread
    shared_ptr<thread>		viewerThread = nullptr;
    const ParameterReader& parameterReader;
    PoseGraph&  poseGraph;

    int    keyframe_size    = 0;
    double resolution       = 0.02;
    double max_distance     = 8.0;
    bool	shutdownFlag	= false;

};


}

#endif // MAPPER_H
