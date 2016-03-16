#include "pose_graph.h"
#include "utils.h"
using namespace rgbd_tutor;

/**
 * @brief PoseGraph::tryInsertKeyFrame
 * @param frame
 * @return
 * TODO: Problem: when inserting large number of keyframes, the mapping thread will block for long time, causing even more key-frames to be processed.
 */
bool PoseGraph::tryInsertKeyFrame(RGBDFrame::Ptr& frame)
{
    if ( keyframes.size() == 0 )
    {
        // 图是空的，直接加入原始点
        unique_lock<mutex> lck(keyframes_mutex);
        keyframes.push_back(frame);
        refFrame = frame;
        g2o::VertexSE3* v = new g2o::VertexSE3();
        v->setId( frame->id );
        v->setEstimate( frame->T_f_w.inverse() );
        v->setFixed(true);
        optimizer.addVertex( v );
        vertexIdx.push_back( frame->id );
        return true;
    }

    // 计算 frame 和 refFrame 之间的位移差
    Eigen::Isometry3d delta = frame->getTransform().inverse() * refFrame->getTransform();
    if ( norm_translate( delta ) > keyframe_min_translation ||
         norm_rotate( delta ) > keyframe_min_rotation )
    {
        // 离keyframe够远
        // 在key frames中进行插入，并在图中生成对应节点和边
        unique_lock<mutex> lck(keyframes_mutex);
        cout<<YELLOW<<"adding keyframe "<<frame->id<<" with ref to "<<refFrame->id<<", n_t="<<norm_translate( delta )<<",n_r="<<norm_rotate(delta)<<RESET<<endl;
        newFrames.push_back( frame );
        
        //  add the vertex
        g2o::VertexSE3* v = new g2o::VertexSE3();
        v->setId( frame->id );
        v->setEstimate( frame->getTransform().inverse() );
        v->setFixed(false);
        optimizer.addVertex( v );
        vertexIdx.push_back( frame->id );
        keyframes.push_back( frame );

        // and the edge with refframe
        // 这里直接根据refFrame和currentFrame的位姿差生成一个边
        // 因为位姿差是tracker估计出来的，我们认为这是比较准的
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        // 注意边的赋值有些绕，详见EdgeSE3的误差计算方式
        g2o::VertexSE3* v0 = dynamic_cast<g2o::VertexSE3*> (optimizer.vertex( refFrame->id ));
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*> (optimizer.vertex( frame->id ));
        edge->setVertex(0, v1);
        edge->setVertex(1, v0);
        // because the state is estimated from tracker
        edge->setMeasurementFromState();
        edge->setInformation( Eigen::Matrix<double,6,6>::Identity() * 100);
        edge->setRobustKernel( new g2o::RobustKernelHuber() );

        EdgeID id;
        id[refFrame->id] = frame->id;
        edges[ id ] = edge;
        optimizer.addEdge( edge );
        
        // set ref frame to current
        refFrame = frame;

        keyframe_updated.notify_one();
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief PoseGraph::mainLoop
 */
void PoseGraph::mainLoop()
{
    cout<<"starting pose graph thread..."<<endl;
    double  loopAccumulatedError = 0.0; //回环的累积误差
    double  localAccumulatedError = 0.0; //回环的累积误差
    while(1)
    {
        if (shutDownFlag == true)
        {
            break;
        }
        unique_lock<mutex> lck_update_keyframe(keyframe_updated_mutex);
        keyframe_updated.wait( lck_update_keyframe );    //等到keyframes有更新
        cout<<"keyframes are updated"<<endl;
        boost::timer timer;
        // 复制一份newFrames，防止处理的时候有新的东西插入
        unique_lock<mutex> lck(keyframes_mutex);
        vector<RGBDFrame::Ptr>  newFrames_copy = newFrames;
        newFrames.clear();
        

        bool    findLargeLoop = false;
        // 检测新增的keyframe并检测其中的回环
        // 边
        cout<<"new key frames = "<<newFrames_copy.size()<<endl;
        
        for ( auto nf : newFrames_copy )
        {
            // 检测nf和keyframes末尾几个的关系
            // 请注意 事实上neframes里的东西已经出现在keyframes里边了
            for ( int i=0; i<nearbyFrames; i++ )
            {
                int idx = keyframes.size()-i-2;
                if (idx < 0)
                {
                    break;
                }
                RGBDFrame::Ptr pf = keyframes[idx];
                //cout<<"checking "<<nf->id<<" and "<<pf->id<<endl;
                //  检测边是否存在
                if (isEdgeExist( nf->id, pf->id ))
                {
                    continue;
                }

                // 用pnp检测nf和pf之间是否可以计算一个边
                PNP_INFORMATION info;
                if ( pnp->solvePnPLazy( pf, nf, info, false ) == false )
                {
                    continue;
                }
                // pnp成功，将pnp结果加到graph中
                cout<<"solve pnp ok, generating an edge"<<endl;
                g2o::EdgeSE3* edge = new g2o::EdgeSE3();
                edge->vertices()[0] = dynamic_cast<g2o::VertexSE3*> (optimizer.vertex( nf->id ));
                edge->vertices()[1] = dynamic_cast<g2o::VertexSE3*> (optimizer.vertex( pf->id ));
                edge->setMeasurement( info.T );
                edge->setInformation( Eigen::Matrix<double,6,6>::Identity() * 100);
                edge->setRobustKernel( new g2o::RobustKernelHuber() );
                
                edge->computeError();
                cout<<"add local error "<<edge->chi2()<<endl;
                localAccumulatedError += edge->chi2();
                EdgeID id;
                id[nf->id] = pf->id;
                edges[ id ] = edge;
                optimizer.addEdge( edge );
                cout<<"edge has been added"<<endl;
            }// end of for nearbyFrames

            // nf 的回环检测
            looper->add( nf );
            vector<RGBDFrame::Ptr>  possibleLoops = looper->getPossibleLoops( nf );

            for ( auto pf:possibleLoops )
            {
                if ( isEdgeExist( nf->id, pf->id ) ) //这条边已经存在
                    continue;
                PNP_INFORMATION info;
                if ( pnp->solvePnPLazy( pf, nf, info, false) == true )
                {
                    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
                    edge->vertices()[0] = dynamic_cast<g2o::VertexSE3*> (optimizer.vertex( nf->id ));
                    edge->vertices()[1] = dynamic_cast<g2o::VertexSE3*> (optimizer.vertex( pf->id ));
                    edge->setMeasurement( info.T );
                    edge->setInformation( Eigen::Matrix<double,6,6>::Identity() * 100);
                    edge->setRobustKernel( new g2o::RobustKernelHuber() );
                    //edges.push_back( edge );
                    EdgeID id;
                    id[nf->id] = pf->id;
                    edges[ id ] = edge;
                    optimizer.addEdge( edge );

                    edge->computeError();
                    loopAccumulatedError += edge->chi2();
                }
            } // end of for possible loops
        } // end of for new frames

        // 处理优化
        bool doOptimize = false;
        if ( loopAccumulatedError > loopAccuError )
        {
            // 处理全局优化
            for ( auto v:vertexIdx )
            {
                optimizer.vertex(v)->setFixed(false);
            }
            optimizer.vertex( vertexIdx[0] )->setFixed(true);
            cout<<"global optimization"<<endl;
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            // 重置keyframes和refFrame
            for ( auto kf : keyframes )
            {
                g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*> ( optimizer.vertex( kf->id ) );
                if ( v )
                {
                    kf->setTransform( v->estimate().inverse());
                }
            }
            localAccumulatedError = 0;
            loopAccumulatedError  = 0;
            doOptimize = true;
        }
        else if ( localAccumulatedError > localAccuError )
        {
            // 处理局部优化
            for ( auto v:vertexIdx )
            {
                optimizer.vertex( v )->setFixed( true );
            }
            for ( int i=vertexIdx.size()-1; i>0 && i>vertexIdx.size()-6; i-- )
            {
                optimizer.vertex( vertexIdx[i] )->setFixed( false );
            }
            optimizer.vertex( vertexIdx[0] )->setFixed(true);
            cout<<"local optimization"<<endl;
            
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            // 重置
            for ( int i=keyframes.size()-1; i>0 && i>keyframes.size()-6; i-- )
            {
                //cout<<"i="<<i<<", keyframe size="<<keyframes.size()<<endl;
                g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*> ( optimizer.vertex( keyframes[i]->id ) );
                if ( v )
                {
                    keyframes[i]->setTransform( v->estimate().inverse() );
                }
                
            }
            localAccumulatedError = 0;
            doOptimize = true;
        } // end of if loop accu
        
        if ( doOptimize == true )
        {
            refFrame = keyframes.back();
            tracker->adjust( refFrame );
        }
    }
    cout<<"pose graph thread stops"<<endl;
}
