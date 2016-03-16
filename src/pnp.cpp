#include "pnp.h"
#include "converter.h"
#include "orb.h"

bool rgbd_tutor::PnPSolver::solvePnP( const vector<cv::Point2f>& img, const vector<cv::Point3f>& obj, const rgbd_tutor::CAMERA_INTRINSIC_PARAMETERS& camera, vector<int>& inliersIndex, Eigen::Isometry3d& transform )
{
    // g2o初始化
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType> ();
    g2o::BlockSolver_6_3*   solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
    optimizer.setAlgorithm( solver );

    // 加入待估计的位姿：一个se3 vertex
    g2o::VertexSE3Expmap*   vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate( rgbd_tutor::Converter::toSE3Quat( Eigen::Isometry3d::Identity()) );
    vSE3->setFixed( false );
    // id为零
    vSE3->setId(0);
    optimizer.addVertex( vSE3 );

    // 接下来就是一堆边，边类型为se3 project xyz only pose
    // 这种边只有一个端点，就是se3本身
    // 先用一个vector装起来，之后要用
    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> edges;

    // 谜之delta
    const float delta = sqrt(5.991);
    // 每条边是否为inliers
    vector<bool>    inliers( img.size(), true );
    int good = 0;
    for ( size_t i=0; i<obj.size(); i++ )
    {
        if (obj[i] == cv::Point3f(0,0,0))
        {
            // 该点值不存在
            inliers[i] = false;
            continue;
        }
        good++;

        g2o::EdgeSE3ProjectXYZOnlyPose * edge = new g2o::EdgeSE3ProjectXYZOnlyPose();
        edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        // 2D点通过setmeasurement来设
        edge->setMeasurement( Eigen::Vector2d(img[i].x, img[i].y) );
        // 这种edge比较特殊，3D点和相机参数直接为成员变量
        edge->fx = camera.fx;
        edge->fy = camera.fy;
        edge->cx = camera.cx;
        edge->cy = camera.cy;
        edge->Xw = Eigen::Vector3d( obj[i].x, obj[i].y, obj[i].z );
        // information其实没多大意义，但为了能求解还是要设一个
        edge->setInformation( Eigen::Matrix2d::Identity()*1 );
        // 由于误匹配的存在，要设置robust kernel
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber();
        edge->setRobustKernel( rk );
        rk->setDelta( delta );
        optimizer.addEdge( edge );
        edge->setId( i );
        edges.push_back(edge);
    }

    // 使用g2o来判断inliers
    // 一共进行四轮迭代，每轮迭代十次
    for(size_t it=0; it<4; it++)
    {
        vSE3->setEstimate( rgbd_tutor::Converter::toSE3Quat( transform ) );
        optimizer.initializeOptimization(0);
        optimizer.optimize( 10 );

        for ( size_t i=0; i<edges.size(); i++ )
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = edges[i];
            if ( inliers[ e->id() ] == true )
            {
                e->computeError();
            }
            //如果某条边的均方误差太大，说明它是一条outlier
            if ( e->chi2() > 5.991 )
            {
                inliers[ e->id() ] = false;
                e->setLevel(1);
                good -- ;
            }
            else
            {
                // 否则就是inlier
                inliers[i] = true;
                e->setLevel(0);
            }

            // 去掉较大误差的边后，就不必再用robust kernel了
            if (it==2)
                e->setRobustKernel( nullptr );
        }

        // 如果inlier太少，就中断
        if (good < 5)
            break;
    }

    for ( size_t i=0; i<inliers.size(); i++ )
    {
        if ( inliers[i] )
        {
            inliersIndex.push_back(i);
        }
    }

    g2o::VertexSE3Expmap* vSE_recov = dynamic_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(0));
    g2o::SE3Quat    se3_recov = vSE_recov->estimate();

    transform = Eigen::Isometry3d( se3_recov );

    if (inliers.size() > min_inliers)
        return true;
    return false;
}

bool rgbd_tutor::PnPSolver::solvePnPLazy( const rgbd_tutor::RGBDFrame::Ptr & frame1, const rgbd_tutor::RGBDFrame::Ptr frame2, PNP_INFORMATION& pnp_information, bool drawMatches )
{
    vector<cv::DMatch>  matches = orb.match( frame1, frame2 );
    if ( matches.size() <= min_match )
        return false;
    vector<cv::Point3f> obj;
    vector<cv::Point2f> img;

    vector<cv::DMatch>  validMatches;

    for (auto m:matches)
    {
        cv::Point3f pObj = frame1->features[m.queryIdx].position;
        if (pObj == cv::Point3f(0,0,0))
            continue;
        if (drawMatches)
        {
            validMatches.push_back(m);
        }

        obj.push_back( pObj );
        img.push_back( frame2->features[m.trainIdx].keypoint.pt );
    }

    if ( img.size() <= min_match || obj.size() <= min_match )
        return false;

    vector<int> inliersIndex;
    Eigen::Isometry3d   init_transform = frame1->T_f_w.inverse() * frame2->T_f_w;
    //cout<<"init transform = "<<init_transform.matrix()<<endl;
    bool b = solvePnP( img, obj, frame1->camera, inliersIndex, init_transform );

    pnp_information.numFeatureMatches = img.size();
    pnp_information.numInliers = inliersIndex.size();
    pnp_information.T = init_transform;

    if (drawMatches == true && b==true)
    {
        vector<cv::DMatch> inlierMatches;
        for ( int index:inliersIndex )
            inlierMatches.push_back( validMatches[index] );
        cv::Mat out;
        cv::drawMatches(frame1->rgb, frame1->getAllKeypoints(),
                        frame2->rgb, frame2->getAllKeypoints(),
                        inlierMatches, out );
        cv::imshow( "inlier matches", out );
    }

    if ( pnp_information.numInliers < min_inliers )
    {
        return false;
    }
    return true;

}
