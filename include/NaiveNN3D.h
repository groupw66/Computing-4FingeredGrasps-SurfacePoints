#ifndef NAIVENN3D_H
#define NAIVENN3D_H
#include <vector>
#include <Eigen/Dense>


class NaiveNN3D
{
    public:
        NaiveNN3D();
        virtual ~NaiveNN3D();
        NaiveNN3D(const std::vector<Eigen::Vector3d> &points);
        int initPoints(const std::vector<Eigen::Vector3d> &points);
        int knnSearch(const std::vector<Eigen::Vector3d> &points,
                      std::vector< std::vector< std::pair<Eigen::Vector3d, double> > > &out, int knn=1);

        std::vector<Eigen::Vector3d> dataset;
    protected:
    private:
};

#endif // NAIVENN3D_H
