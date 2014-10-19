#ifndef POSITIONSNORMALSFILE_H
#define POSITIONSNORMALSFILE_H

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>

class PositionsNormalsFile
{
    public:
        PositionsNormalsFile();
        virtual ~PositionsNormalsFile();
        PositionsNormalsFile(const char *_filename);
        void open(const char *_filename);

        char filename[1000];
        std::vector<Eigen::Vector3d> positions;
        std::vector<Eigen::Vector3d> normals;

    protected:
    private:
};

#endif // POSITIONSNORMALSFILE_H
