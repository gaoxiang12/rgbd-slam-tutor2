#include "common_headers.h"
#include "rgbdframe.h"
#include "track.h"

using namespace std;
using namespace rgbd_tutor;

int main()
{
    cout<<"running tracker..."<<endl;
    ParameterReader para;
    FrameReader frameReader( para );
    Tracker tracker(para);

    while ( RGBDFrame::Ptr frame = frameReader.next() )
    {
        cout<<"*************************************"<<endl;
        cout<<"tracking frame "<<frame->id<<endl;
        boost::timer    timer;
        Eigen::Isometry3d T = tracker.updateFrame( frame );
        cout<<"current T="<<endl<<T.matrix()<<endl;
        cv::imshow( "image", frame->rgb );
        if (tracker.getState() == Tracker::LOST)
        {
            cout<<"The tracker has lost"<<endl;
            cv::waitKey(0);
        }
        else
        {
            cv::waitKey(1);
        }
        cout<<"time cost = "<<timer.elapsed()<<endl;
    }
    return 0;
}
