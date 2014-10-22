#include "PositionsNormalsFile.h"

PositionsNormalsFile::PositionsNormalsFile()
{
    //ctor
}

PositionsNormalsFile::~PositionsNormalsFile()
{
    //dtor
}
PositionsNormalsFile::PositionsNormalsFile(const char* _filename)
{
    open(_filename);
}

void PositionsNormalsFile::open(const char* _filename)
{
    positions.clear();
    normals.clear();
    std::ifstream myfile(_filename);
    int n;
    myfile >> n;
    positions.reserve(n);
    normals.reserve(n);
    for(int i=0 ; i<n ; ++i){
        double a,b,c,d,e,f;
        myfile >> a >> b >> c >> d >> e >> f;
        Eigen::Vector3d p(a,b,c);
        positions.push_back(p);
        Eigen::Vector3d n(-d,-e,-f);
        normals.push_back(n);
    }
}
