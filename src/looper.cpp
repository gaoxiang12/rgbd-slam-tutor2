#include "looper.h"

using namespace rgbd_tutor;

vector<RGBDFrame::Ptr> Looper::getPossibleLoops( const RGBDFrame::Ptr& frame )
{
    vector<RGBDFrame::Ptr>  result;
    for ( size_t i=0; i<frames.size(); i++ )
    {
        RGBDFrame::Ptr pf = frames[i];
        double  score = vocab.score( frame->bowVec, pf->bowVec );
        if (score > min_sim_score && abs(pf->id-frame->id)>min_interval )
        {
            result.push_back( pf );
        }
    }
    return result;
}
