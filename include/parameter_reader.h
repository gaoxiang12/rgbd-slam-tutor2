#ifndef PARAMETER_READER_H
#define PARAMETER_READER_H

#include "common.h"

namespace rgbd_tutor
{

class ParameterReader
{
public:
    // 构造函数：传入参数文件的路径
    ParameterReader( const string& filename = "./parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            // 看看上级目录是否有这个文件 ../parameter.txt
            fin.open("."+filename);
            if (!fin)
            {
                cerr<<"没有找到对应的参数文件："<<filename<<endl;
                return;
            }
        }

        // 从参数文件中读取信息
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }
            int pos = str.find('#');
            if (pos != -1)
            {
                //从井号到末尾的都是注释
                str = str.substr(0, pos);
            }

            // 查找等号
            pos = str.find("=");
            if (pos == -1)
                continue;
            // 等号左边是key，右边是value
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }

    // 获取数据
    // 由于数据类型不确定，写成模板
    template< class T >
    T getData( const string& key ) const
    {
        auto iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return boost::lexical_cast<T>( "" );
        }
        // boost 的 lexical_cast 能把字符串转成各种 c++ 内置类型
        return boost::lexical_cast<T>( iter->second );
    }

    // 直接返回读取到的相机内参
    rgbd_tutor::CAMERA_INTRINSIC_PARAMETERS getCamera() const
    {
        static rgbd_tutor::CAMERA_INTRINSIC_PARAMETERS camera;
        camera.fx = this->getData<double>("camera.fx");
        camera.fy = this->getData<double>("camera.fy");
        camera.cx = this->getData<double>("camera.cx");
        camera.cy = this->getData<double>("camera.cy");
        camera.d0 = this->getData<double>("camera.d0");
        camera.d1 = this->getData<double>("camera.d1");
        camera.d2 = this->getData<double>("camera.d2");
        camera.d3 = this->getData<double>("camera.d3");
        camera.d4 = this->getData<double>("camera.d4");
        camera.scale = this->getData<double>("camera.scale");
        return camera;
    }

protected:
    map<string, string> data;
};

};

#endif // PARAMETER_READER_H
