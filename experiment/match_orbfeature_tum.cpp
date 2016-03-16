#include "common_headers.h"
#include "parameter_reader.h"
#include "rgbdframe.h"
#include "orb.h"

#include <opencv2/features2d/features2d.hpp>
#include <cstdlib>

#include "Thirdparty/orbslam_modified/include/ORBextractor.h"

using namespace std;
using namespace rgbd_tutor;

int main()
{
    cout<<"running orbfeature_tum"<<endl;
    ParameterReader para;
    FrameReader frameReader( para );
    OrbFeature  orb( para );
    RGBDFrame::Ptr last_frame = frameReader.next();
    while ( RGBDFrame::Ptr frame = frameReader.next() )
    {
        boost::timer    timer;
        orb.detectFeatures( frame );
        vector<cv::DMatch>  matches = orb.match( last_frame, frame );
        cout<<"matches = "<<matches.size()<<endl;
        cout<<"timer used for detecting and matching features"<<timer.elapsed()<<endl;
        last_frame = frame;
    }
    return 0;
}
