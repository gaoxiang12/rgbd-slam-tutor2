#include "rgbdframe.h"
#include "track.h"
#include "pose_graph.h"
#include "mapper.h"
#include "common_headers.h"


using namespace std;
using namespace rgbd_tutor;
int main()
{
    ParameterReader	parameterReader;
    Tracker::Ptr	tracker( new Tracker(parameterReader) );
    FrameReader		frameReader( parameterReader );
    PoseGraph		poseGraph( parameterReader, tracker );
    Mapper			mapper( parameterReader, poseGraph );

    while ( RGBDFrame::Ptr frame = frameReader.next() )
    {
        boost::timer timer;
        cv::imshow("image", frame->rgb);
        cv::waitKey(1);
        tracker->updateFrame( frame );
        poseGraph.tryInsertKeyFrame( frame );
        
        if (tracker->getState() == Tracker::LOST)
        {
            cout<<"tracker is lost"<<endl;
            //break;
        }
        cout<<"cost time = "<<timer.elapsed()<<endl;
    }

    poseGraph.shutdown();
    mapper.shutdown();

    return 0;
}
