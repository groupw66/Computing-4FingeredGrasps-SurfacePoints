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

bool PositionsNormalsFile::open(const char* _filename)
{
    positions.clear();
    normals.clear();
    std::ifstream myfile(_filename);
    if(!myfile.is_open())
        return false;
    int n;
    myfile >> n;
    positions.reserve(n);
    normals.reserve(n);
    for(int i=0 ; i<n ; ++i){
        double a,b,c,d,e,f;
        myfile >> a >> b >> c >> d >> e >> f;
        Eigen::Vector3d p(a,b,c);
        positions.push_back(p);
        //Eigen::Vector3d n(-d,-e,-f);
        Eigen::Vector3d n(d,e,f);
        normals.push_back(n);
    }
    myfile.close();
    return true;
}

PositionsNormalsFile::PositionsNormalsFile(std::vector<Eigen::Vector3d> _positions, std::vector<Eigen::Vector3d> _normals)
{
    positions = _positions;
    normals = _normals;
}

bool PositionsNormalsFile::write(const char* _filename)
{
    if(positions.size() != normals.size())
        return false;
    std::ofstream myfile(_filename);
    myfile.unsetf ( std::ios::floatfield );
    myfile.precision(std::numeric_limits<long double>::digits10);
    if(!myfile.is_open())
        return false;
    myfile << positions.size() << "\n";
    for(unsigned int i=0 ; i<positions.size() ; ++i){
        myfile << positions[i].x() << " " << positions[i].y() << " " << positions[i].z() << " "
                << normals[i].x() << " " << normals[i].y() << " " << normals[i].z() << "\n";
    }
    myfile.close();
    return true;
}

