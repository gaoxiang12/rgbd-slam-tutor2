#include "pnp.h"
#include "common_headers.h"

using namespace std;
using namespace rgbd_tutor;

int main()
{
    cout<<"running test pnp"<<endl;
    ParameterReader para;
    FrameReader frameReader( para );
    OrbFeature  orb(para);
    PnPSolver   pnp(para, orb);

    RGBDFrame::Ptr refFrame = frameReader.next();
    orb.detectFeatures( refFrame );
    Eigen::Isometry3d   speed = Eigen::Isometry3d::Identity();

    while (1)
    {
        cout<<"*************************************"<<endl;
        boost::timer timer;
        RGBDFrame::Ptr currFrame = frameReader.next();

        if ( currFrame == nullptr )
        {
            break;
        }
        currFrame->T_f_w = speed * refFrame->T_f_w ;
        orb.detectFeatures( currFrame );

        PNP_INFORMATION info;
        bool result = pnp.solvePnPLazy( refFrame, currFrame, info, true );

        if ( result == false )
        {
            cout<<"pnp failed"<<endl;
            refFrame = currFrame;
            cv::waitKey(0);
        }
        else
        {
            currFrame->T_f_w = info.T * refFrame->T_f_w;
            cout<<"result.T="<<endl;
            cout<<info.T.matrix()<<endl;
            cout<<"current = "<< endl << currFrame->T_f_w.matrix() << endl;
            speed = info.T;
            refFrame = currFrame;
            cv::waitKey(1);
        }

        cout<<GREEN<<"time used = "<<timer.elapsed()<<RESET<<endl;

    }

    return 0;
}
