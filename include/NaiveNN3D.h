#ifndef NAIVENN3D_H
#define NAIVENN3D_H
#include <vector>
#include <Eigen/Dense>
#include "NearestNeighbor.h"


class NaiveNN3D : public NearestNeighbor
{
    public:
        NaiveNN3D();
        virtual ~NaiveNN3D();
        NaiveNN3D(const std::vector<Eigen::Vector3d> &points) { initPoints(points); }
        int initPoints(const std::vector<Eigen::Vector3d> &points);
        int knnSearch(const std::vector<Eigen::Vector3d> &points,
                      std::vector< std::vector< std::tuple<Eigen::Vector3d, int, double> > > &out, int knn=1);
        int knnSearch(const Eigen::Vector3d &point,
                      std::vector< std::tuple<Eigen::Vector3d, int, double> > &out, int knn=1);
        std::vector<Eigen::Vector3d> dataset;
    protected:
    private:
};

#endif // NAIVENN3D_H
