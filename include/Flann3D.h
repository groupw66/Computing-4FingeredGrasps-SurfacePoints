#ifndef FLANN3D_H
#define FLANN3D_H
#include <flann/flann.hpp>
#include <vector>
#include <Eigen/Dense>
#include "NearestNeighbor.h"


class Flann3D : public NearestNeighbor
{
    public:
        Flann3D();
        virtual ~Flann3D();
        Flann3D(const std::vector<Eigen::Vector3d> &points) { initPoints(points); }
        int initPoints(const std::vector<Eigen::Vector3d> &points);
        int knnSearch(const std::vector<Eigen::Vector3d> &points,
                      std::vector< std::vector< std::tuple<Eigen::Vector3d, int, double> > > &out, int knn=1);
        int knnSearch(const Eigen::Vector3d &point,
                      std::vector< std::tuple<Eigen::Vector3d, int, double> > &out, int knn=1);

        std::vector<Eigen::Vector3d> tmpPoints;
        flann::Matrix<double> dataset;
        flann::Index<flann::L2<double> > index = flann::Index<flann::L2<double> >(flann::KDTreeSingleIndexParams(10));
    protected:
    private:
};

#endif // FLANN3D_H
