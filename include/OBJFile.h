#ifndef OBFFILE_H
#define OBFFILE_H

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>

class OBJFile
{
    public:
        OBJFile();
        virtual ~OBJFile();
        OBJFile(const char *_filename);
        void open(const char *_filename);

        char filename[1000];
        std::vector<Eigen::Vector3d> vertexs;
        std::vector<Eigen::Vector3i> facets;

    protected:
    private:
};

#endif // OBFFILE_H
