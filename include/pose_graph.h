#ifndef POSE_GRAPH_H
#define POSE_GRAPH_H
#include "common_headers.h"
#include "parameter_reader.h"
#include "rgbdframe.h"
#include "looper.h"
#include "pnp.h"
#include "orb.h"
#include "track.h"

#include <thread>
#include <mutex>
#include <functional>
#include <map>
#include <condition_variable>

/**
  * The pose graph performs global optimization.
  * 在tracker跟踪当前帧currentFrame，返回一个粗略的位姿时，会把这个帧尝试放入pose graph中.
  * pose graph将它与自己的refframe比较，得到一个相对位移估计。当这个估计大于给定阈值时，将该帧作为新的frame插入到pose graph中。
  * pose graph本身有一个优化线程，负责查找相近节点的边并进行优化。
  * 同时它有回环检测模块。检测到大型回环时进行全局优化。
  * 
  * 进行优化时，关键帧序列的位姿会被替换成优化后的值，因此参考帧会发生改变。而tracker存在漂移。
  * 为了使机器人得到全局准确的位姿，需要用pose graph优化后的结果校正tracker。
  */

namespace rgbd_tutor
{
using namespace rgbd_tutor;

class Tracker;

class PoseGraph
{
public:
    PoseGraph( const ParameterReader& para,
               shared_ptr<Tracker>& t )
        : parameterReader( para ),
          tracker( t )
    {
        looper  =   make_shared<Looper>( para );
        orb     =   make_shared<OrbFeature>( para );
        pnp     =   make_shared<PnPSolver>( para, *orb );

        keyframe_min_translation = para.getData<double>("keyframe_min_translation");
        keyframe_min_rotation = para.getData<double>("keyframe_min_rotation");
        nearbyFrames = para.getData<int>("nearby_keyframes");
    
        loopAccuError = para.getData<double>("loop_accumulate_error");                
        localAccuError = para.getData<double>("local_accumulate_error");                

        posegraphThread = make_shared<std::thread> (
                    std::bind(&PoseGraph::mainLoop, this) );

        g2o::LinearSolverCholmod<g2o::BlockSolver<g2o::BlockSolverTraits<6,3> >::PoseMatrixType > * linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver<g2o::BlockSolverTraits<6,3> >::PoseMatrixType > ();

        //g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType> ();
        linearSolver->setBlockOrdering( false );

        g2o::BlockSolver< g2o::BlockSolverTraits<6,3> >* solver = new g2o::BlockSolver< g2o::BlockSolverTraits<6,3> >( linearSolver );
        //g2o::BlockSolver_6_3*   solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg* algo = new g2o::OptimizationAlgorithmLevenberg( solver );
        optimizer.setAlgorithm( algo );
        optimizer.setVerbose(false);

    }

    // 试图插入一个新的keyframe，失败返回false
    bool    tryInsertKeyFrame( RGBDFrame::Ptr& frame );

    //主线程
    void    mainLoop();

    void    shutdown()
    {
        shutDownFlag = true;
        keyframe_updated.notify_all();
        cout<<"please wait pose graph thread to stop..."<<endl;
        if (posegraphThread != nullptr)
        {
            posegraphThread->join();
        }

        // save the results
        cout<<RED<<"saving trajectory"<<endl;
        cout<<"vertex: "<<optimizer.vertices().size()<<endl;
        cout<<"edges: "<<optimizer.edges().size()<<endl;
        save( "./data/traj.g2o" );
        cout<<"trajectory ok."<<RESET<<endl;

        if ( optimizer.vertices().size() > 5)
        {
            // 太少了就不优化了
            optimizer.initializeOptimization();
            optimizer.optimize( 10 );
        }
    }

    void save( const string& filename )
    {
        ofstream fout(filename);
        /*
        for ( size_t i=0; i< keyframes.size(); i++ )
        {
            g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*> (optimizer.vertices()[ keyframes[i]->id ]);
            if (v == nullptr)
            {
                cerr<<"vertex "<<keyframes[i]->id<<" does not exist!"<<endl;
                continue;
            }
            double data[7] = {0};
            v->getEstimateData( data );
            fout<<"VERTEX_SE3:QUAT "<<v->id()<<" ";
            for ( double d:data )
                fout<<d<<" ";
            fout<<endl;
        }

        for ( size_t i=0; i<edges.size(); i++ )
        {
            g2o::EdgeSE3 edge = *edges[i];
            fout<<"EDGE_SE3:QUAT "<<edge.vertices()[0]->id()<<" "<<edge.vertices()[1]->id()<<" ";
            double data[7] = {0};
            edge.getMeasurementData( data );
            for ( double d:data )
                fout<<d<<" ";
            fout<<"100 0 0 0 0 0 100 0 0 0 0 100 0 0 0 100 0 0 100 0 100"<<endl;
        }
        */
        if (fout)
        {
            optimizer.save(filename.c_str());
            fout.close();
        }
    }

protected:
    //  检测限定端点的边是否存在
    bool    isEdgeExist( const int vertex1, const int vertex2 ) const
    {
        if ( vertex1==vertex2 )
            return true;
        EdgeID e1, e2;
        e1[vertex1] = vertex2; e2[vertex2]=vertex1;
        return edges.find( e1 ) != edges.end() ||
                edges.find( e2 ) != edges.end() ;
    }

public:
    //数据
    vector<RGBDFrame::Ptr>  keyframes;
    vector<RGBDFrame::Ptr>  newFrames;      //新关键帧的缓冲区
    RGBDFrame::Ptr          refFrame;       //参考

    std::condition_variable keyframe_updated;
    std::mutex              keyframe_updated_mutex; 
    
    bool                    shutDownFlag    =false;
    std::mutex              keyframes_mutex;

    // 有用的loop之类的东西
    shared_ptr<Looper>      looper      =nullptr;
    shared_ptr<PnPSolver>   pnp         =nullptr;
    shared_ptr<OrbFeature>  orb         =nullptr;

    // pose graph 线程
    shared_ptr<std::thread> posegraphThread =nullptr;

    // tracker 调整当前的姿态
    shared_ptr<Tracker>     tracker     =nullptr;

    // g2o的优化器
    g2o::SparseOptimizer    optimizer;

    // 点集与边集
    vector<int>             vertexIdx;
    typedef map<int,int>    EdgeID;
    map< EdgeID, g2o::EdgeSE3* > edges;

    // 参数读取类
    const   ParameterReader&    parameterReader;

    double  keyframe_min_translation        =0.3;
    double  keyframe_min_rotation           =0.3;
    int     nearbyFrames                    =2;
    int     lastGraphSize                   =0;
    double  loopAccuError                   =1.0;
    double  localAccuError                  =1.0;

};

}
#endif // POSE_GRAPH_H
