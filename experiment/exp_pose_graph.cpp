#include "track.h"
#include "pose_graph.h"
using namespace rgbd_tutor;

int main()
{
    ParameterReader	parameterReader;
    Tracker::Ptr	tracker( new Tracker(parameterReader) );
    FrameReader		frameReader( parameterReader );
    PoseGraph		poseGraph( parameterReader, tracker );

    while( RGBDFrame::Ptr frame = frameReader.next() )
    {
        cout<<"*******************************************"<<endl;
        boost::timer timer;
        cout<<"loading frame "<<frame->id<<endl;
        Eigen::Isometry3d T = tracker->updateFrame( frame );
        cout<<"current frame T = "<<endl<<T.matrix()<<endl;
        cv::imshow( "image", frame->rgb );
        if ( poseGraph.tryInsertKeyFrame( frame ) == true )
        {
            cout<<"Insert key-frame succeed"<<endl;
            cv::waitKey(1);
        }
        else
        {
            cout<<"Insert key-frame failed"<<endl;
            cv::waitKey(1);
        }
        cout<<GREEN<<"time cost="<<timer.elapsed()<<RESET<<endl;
    }

    poseGraph.shutdown();
    return 0;
}
