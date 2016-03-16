#ifndef FRAME_H
#define FRAME_H

#include "common_headers.h"
#include "parameter_reader.h"
#include "feature.h"
#include "utils.h"

#include"Thirdparty/DBoW2/DBoW2/FORB.h"
#include"Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <mutex>

/**
 * RGBDFrame 
 * 该类记录了每个帧的信息。帧是slam的基本单位。
 * 本身可以看作一个struct。
 * 由于Pose可能被若干线程同时访问，需要加锁。
 */

namespace rgbd_tutor{

class RGBDFrame //帧
{
public:
    typedef shared_ptr<RGBDFrame> Ptr;
    // 数据成员
    int id  =-1;                //-1表示该帧不存在
    cv::Mat rgb, depth;
    //从当前帧到世界坐标系的变换
    Eigen::Isometry3d   T_f_w = Eigen::Isometry3d::Identity();
    std::mutex   mutexT;

    // 特征
    vector<Feature>     features;

    // 相机
    CAMERA_INTRINSIC_PARAMETERS camera;

    // BoW回环
    DBoW2::BowVector bowVec;

    //  point cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr	pointcloud =nullptr;

public:
    // 方法
    // 将2d像素点投影到本地3d坐标
    cv::Point3f project2dTo3d( int u, int v  ) const
    {
        if (depth.data == nullptr)
            return cv::Point3f(0,0,0);
        ushort d = depth.ptr<ushort>(v)[u];
        if (d == 0)
            return cv::Point3f(0,0,0);
        cv::Point3f p;
        p.z = double( d ) / camera.scale;
        p.x = ( u - camera.cx) * p.z / camera.fx;
        p.y = ( v - camera.cy) * p.z / camera.fy;
        return p;
    }

    // 将一组descriptor转换为一个矩阵
    cv::Mat getAllDescriptors ( ) const
    {
        cv::Mat desp;
        for ( size_t i=0; i<features.size(); i++ )
        {
            desp.push_back( features[i].descriptor );
        }
        return desp;
    }

    // 获取所有的descriptor并组成一个向量
    vector<cv::Mat> getAllDescriptorsVec() const
    {
        vector<cv::Mat> desp;
        for ( auto f:features )
        {
            desp.push_back( f.descriptor );
        }
        return desp;
    }

    // 获取所有的keypoints
    vector<cv::KeyPoint>    getAllKeypoints() const
    {
        vector<cv::KeyPoint> kps;
        for ( auto f:features )
        {
            kps.push_back( f.keypoint );
        }
        return kps;
    }

    void setTransform( const Eigen::Isometry3d& T )
    {
        std::unique_lock<std::mutex> lck(mutexT);
        T_f_w = T;
    }
    
    Eigen::Isometry3d getTransform()  
    {
        std::unique_lock<std::mutex> lck(mutexT);
        return T_f_w;
    }
};

// FrameReader: 从数据集中顺序读取RGBDFrame
// 现在支持TUM数据集。
class FrameReader
{
public:
    enum DATASET
    {
        NYUD=0,
        TUM=1,
    };

    FrameReader( rgbd_tutor::ParameterReader& para, const DATASET& dataset_type = TUM )
        : parameterReader( para )
    {
        this->dataset_type = dataset_type;

        switch( dataset_type )
        {
        case NYUD:
            //TODO: 实现nyud数据读取接口
            break;
        case TUM:
            init_tum( para );
            break;
        }

        camera = para.getCamera();
    }

    RGBDFrame::Ptr   next();

    void    reset()
    {
        currentIndex = start_index;
    }

    // get by index
    RGBDFrame::Ptr   get( int index )
    {
        if (index < 0 || index >= rgbFiles.size() )
            return nullptr;
        currentIndex = index;
        return next();
    }

protected:
    void    init_tum( ParameterReader& para );
protected:
    DATASET     dataset_type    =TUM;

    int currentIndex =0;
    int start_index  =0;
    vector<string>  rgbFiles, depthFiles;
    string  dataset_dir;
    const ParameterReader&  parameterReader;

    CAMERA_INTRINSIC_PARAMETERS     camera;
};

};
#endif // FRAME_H
