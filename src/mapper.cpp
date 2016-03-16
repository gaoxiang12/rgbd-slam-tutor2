#include "mapper.h"

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

using namespace rgbd_tutor;

Mapper::PointCloud::Ptr Mapper::generatePointCloud(const RGBDFrame::Ptr &frame)
{
    PointCloud::Ptr tmp( new PointCloud() );
    if ( frame->pointcloud == nullptr )
    {
        // point cloud is null ptr
        frame->pointcloud = boost::make_shared<PointCloud>();
#pragma omp parallel for
        for ( int m=0; m<frame->depth.rows; m+=3 )
        {
            for ( int n=0; n<frame->depth.cols; n+=3 )
            {
                ushort d = frame->depth.ptr<ushort>(m)[n];
                if (d == 0)
                    continue;
                if (d > max_distance * frame->camera.scale)
                    continue;
                PointT p;
                cv::Point3f p_cv = frame->project2dTo3d(n, m);
                p.b = frame->rgb.ptr<uchar>(m)[n*3];
                p.g = frame->rgb.ptr<uchar>(m)[n*3+1];
                p.r = frame->rgb.ptr<uchar>(m)[n*3+2];

                p.x = p_cv.x;
                p.y = p_cv.y;
                p.z = p_cv.z;

                frame->pointcloud->points.push_back( p );
            }
        }
    }

    Eigen::Isometry3d T = frame->getTransform().inverse();
    pcl::transformPointCloud( *frame->pointcloud, *tmp, T.matrix());
    tmp->is_dense = false;
    return tmp;
}

void Mapper::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    PointCloud::Ptr globalMap (new PointCloud);

    pcl::VoxelGrid<PointT>	voxel;
    voxel.setLeafSize( resolution, resolution, resolution );

    while (shutdownFlag == false)
    {
        static int cntGlobalUpdate = 0;
        if ( poseGraph.keyframes.size() <= this->keyframe_size )
        {
            usleep(1000);
            continue;
        }
        // keyframe is updated
        PointCloud::Ptr	tmp(new PointCloud());
        if (cntGlobalUpdate % 15 == 0)
        {
            // update all frames
            cout<<"redrawing frames"<<endl;
            globalMap->clear();
            for ( int i=0; i<poseGraph.keyframes.size(); i+=2 )
            {
                PointCloud::Ptr cloud = this->generatePointCloud(poseGraph.keyframes[i]);
                *globalMap += *cloud;
            }
        }
        else
        {
            for ( int i=poseGraph.keyframes.size()-1; i>=0 && i>poseGraph.keyframes.size()-6; i-- )
            {
                PointCloud::Ptr cloud = this->generatePointCloud(poseGraph.keyframes[i]);
                *globalMap += *cloud;
            }
        }

        cntGlobalUpdate ++ ;
        //voxel
        voxel.setInputCloud( globalMap );
        voxel.filter( *tmp );

        keyframe_size = poseGraph.keyframes.size();
        globalMap->swap( *tmp );
        viewer.showCloud( globalMap );

        cout<<"points in global map: "<<globalMap->points.size()<<endl;
    }
}
