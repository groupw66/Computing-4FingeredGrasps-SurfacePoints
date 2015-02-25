#ifndef NEARESTNEIGHBOR_H
#define NEARESTNEIGHBOR_H
#include <vector>
#include <Eigen/Dense>


class NearestNeighbor
{
    public:
        virtual int knnSearch(const std::vector<Eigen::Vector3d> &points,
                      std::vector< std::vector< std::tuple<Eigen::Vector3d, int, double> > > &out, int knn=1) = 0;
        virtual int knnSearch(const Eigen::Vector3d &point,
                      std::vector< std::tuple<Eigen::Vector3d, int, double> > &out, int knn=1) = 0;
    protected:
    private:
};

#endif // NEARESTNEIGHBOR_H
