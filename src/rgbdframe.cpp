#include "rgbdframe.h"
#include "common_headers.h"
#include "parameter_reader.h"

using namespace rgbd_tutor;

RGBDFrame::Ptr   FrameReader::next()
{
    switch (dataset_type) {
    case NYUD:
        //TODO 增加nyud的接口
        break;
    case TUM:
    {
        if (currentIndex < start_index || currentIndex >= rgbFiles.size())
            return nullptr;

        RGBDFrame::Ptr   frame (new RGBDFrame);
        frame->id = currentIndex;
        frame->rgb = cv::imread( dataset_dir + rgbFiles[currentIndex]);
        frame->depth = cv::imread( dataset_dir + depthFiles[currentIndex], -1);

        if (frame->rgb.data == nullptr || frame->depth.data==nullptr)
        {
            // 数据不存在
            return nullptr;
        }

        frame->camera = this->camera;
        currentIndex ++;
        return frame;
    }
    default:
        break;
    }

    return nullptr;
}

void FrameReader::init_tum( ParameterReader& para )
{
    dataset_dir = parameterReader.getData<string>("data_source");
    string  associate_file  =   dataset_dir+"/associate.txt";
    ifstream    fin(associate_file.c_str());
    if (!fin)
    {
        cerr<<"找不着assciate.txt啊！在tum数据集中这尼玛是必须的啊!"<<endl;
        cerr<<"请用python assicate.py rgb.txt depth.txt > associate.txt生成一个associate文件，再来跑这个程序！"<<endl;
        return;
    }

    while( !fin.eof() )
    {
        string rgbTime, rgbFile, depthTime, depthFile;
        fin>>rgbTime>>rgbFile>>depthTime>>depthFile;
        if ( !fin.good() )
        {
            break;
        }
        rgbFiles.push_back( rgbFile );
        depthFiles.push_back( depthFile );
    }

    cout<<"一共找着了"<<rgbFiles.size()<<"个数据记录哦！"<<endl;
    camera = parameterReader.getCamera();
    start_index = parameterReader.getData<int>("start_index");
    currentIndex = start_index;
}
