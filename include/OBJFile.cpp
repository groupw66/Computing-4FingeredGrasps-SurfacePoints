#include "OBJFile.h"

OBJFile::OBJFile()
{
    //ctor
}

OBJFile::~OBJFile()
{
    //dtor
}
OBJFile::OBJFile(const char* _filename)
{
    open(_filename);
}
void OBJFile::open(const char* _filename)
{
    vertexs.clear();
    facets.clear();
    strcpy(filename,_filename);
    std::ifstream myfile(filename);
    while(!myfile.eof())
    {
        std::string line;
        getline(myfile, line);
        std::stringstream ss(line);
        if(line[0] == 'v'){
            double i, j ,k;
            std::string tmp;
            ss >> tmp >> i >> j >> k;
            Eigen::Vector3d vertex(i,j,k);
            vertexs.push_back(vertex);
        }
        else if(line[0] == 'f'){
            std::vector<int> iVertexs;
            std::string item;
            std::getline(ss, item, ' ');
            while (std::getline(ss, item, ' ')) {
                iVertexs.push_back(atoi(item.c_str())-1);
            }
            for(int i=2 ; i<iVertexs.size() ; ++i){
                facets.push_back(Eigen::Vector3i(iVertexs[0], iVertexs[i-1], iVertexs[i]));
            }
        }
    }
}
