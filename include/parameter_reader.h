#ifndef PARAMETER_READER_H
#define PARAMETER_READER_H
#include "common_headers.h"

namespace rgbd_tutor
{
struct CAMERA_INTRINSIC_PARAMETERS;

class ParameterReader
{
public:
    ParameterReader( string filename="./parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            // try ../parameter.txt
            fin.open("../parameters.txt");
            if (!fin)
            {
                cerr<<"parameter file does not exist."<<endl;
                return;
            }
        }
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
            pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }

    template< class T >
    T getData( const string& key ) const
    {
        auto iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
        }
        return boost::lexical_cast<T>( iter->second );
    }

    rgbd_tutor::CAMERA_INTRINSIC_PARAMETERS getCamera() const;

public:
    map<string, string> data;
};

};
#endif // PARAMETER_READER_H
