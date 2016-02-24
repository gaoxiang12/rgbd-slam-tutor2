#ifndef RGBDFRAME_H
#define RGBDFRAME_H

#include "common.h"
#include "parameter_reader.h"

#include"Thirdparty/DBoW2/DBoW2/FORB.h"
#include"Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace rgbd_tutor{

//帧
class RGBDFrame
{
public:
    typedef shared_ptr<RGBDFrame> Ptr;


public:
    RGBDFrame() {}
    // 方法
    // 给定像素点，求3D点坐标
    cv::Point3f project2dTo3dLocal( const int& u, const int& v  ) const
    {
        if (depth.data == nullptr)
            return cv::Point3f();
        ushort d = depth.ptr<ushort>(v)[u];
        if (d == 0)
            return cv::Point3f();
        cv::Point3f p;
        p.z = double( d ) / camera.scale;
        p.x = ( u - camera.cx) * p.z / camera.fx;
        p.y = ( v - camera.cy) * p.z / camera.fy;
        return p;
    }

public:
    // 数据成员
    int id  =-1;            //-1表示该帧不存在

    // 彩色图和深度图
    cv::Mat rgb, depth;
    // 该帧位姿
    // 定义方式为：x_local = T * x_world 注意也可以反着定义；
    Eigen::Isometry3d       T=Eigen::Isometry3d::Identity();

    // 特征
    vector<cv::KeyPoint>    keypoints;
    cv::Mat                 descriptor;

    // 相机
    // 默认所有的帧都用一个相机模型（难道你还要用多个吗？）
    CAMERA_INTRINSIC_PARAMETERS camera;

    // BoW回环特征
    // 讲BoW时会用到，这里先请忽略之
    DBoW2::BowVector bowVec;

};

// FrameReader
// 从TUM数据集中读取数据的类
class FrameReader
{
public:
    FrameReader( const rgbd_tutor::ParameterReader& para )
        : parameterReader( para )
    {
        init_tum( );
    }

    // 获得下一帧
    RGBDFrame::Ptr   next();

    // 重置index
    void    reset()
    {
        cout<<"重置 frame reader"<<endl;
        currentIndex = start_index;
    }

    // 根据index获得帧
    RGBDFrame::Ptr   get( const int& index )
    {
        if (index < 0 || index >= rgbFiles.size() )
            return nullptr;
        currentIndex = index;
        return next();
    }

protected:
    // 初始化tum数据集
    void    init_tum( );
protected:

    // 当前索引
    int currentIndex =0;
    // 起始索引
    int start_index  =0;

    const   ParameterReader&    parameterReader;

    // 文件名序列
    vector<string>  rgbFiles, depthFiles;

    // 数据源
    string  dataset_dir;

    // 相机内参
    CAMERA_INTRINSIC_PARAMETERS     camera;
};

};
#endif // RGBDFRAME_H
