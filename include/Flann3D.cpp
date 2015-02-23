#include "Flann3D.h"

Flann3D::Flann3D()
{
    //ctor
}

Flann3D::~Flann3D()
{
    //dtor
}

Flann3D::Flann3D(const std::vector<Eigen::Vector3d> &points)
{
    initPoints(points);
}

int Flann3D::initPoints(const std::vector<Eigen::Vector3d> &points)
{
    std::vector<Eigen::Vector3d> tmpPoints = points;
    dataset = flann::Matrix<double>(tmpPoints[0].data(), points.size(), 3);
    index.buildIndex(dataset);
    return 0;
}

int Flann3D::knnSearch(const std::vector<Eigen::Vector3d>& points, std::vector< std::vector< std::pair<Eigen::Vector3d, double> > >& out, int knn)
{
    std::vector<Eigen::Vector3d> tmpPoints = points;
    flann::Matrix<double> query = flann::Matrix<double>(tmpPoints[0].data(), points.size(), 3);
    int nQuery = points.size();
    std::vector< std::vector<int> > indices;
    std::vector< std::vector<double> > dists;
    index.knnSearch(query, indices, dists, knn, flann::SearchParams(128));
    out.resize(nQuery);
    for(int i=0 ; i < nQuery ; ++i){
        out[i].resize(knn);
        for(int j=0 ; j < knn ;++j){
            double* position = index.getPoint(indices[i][j]);
            out[i][j] = std::make_pair(Eigen::Vector3d(position[0], position[1], position[2]), sqrt(dists[i][j]));
        }
    }
    return 0;
}

int Flann3D::knnSearch(const Eigen::Vector3d& point, std::vector< std::pair<Eigen::Vector3d, double> >& out, int knn)
{
    Eigen::Vector3d tmpPoint = point;
    flann::Matrix<double> query = flann::Matrix<double>(tmpPoint.data(), 1, 3);
    std::vector< std::vector<int> > indices;
    std::vector< std::vector<double> > dists;
    index.knnSearch(query, indices, dists, knn, flann::SearchParams(128));
    out.resize(knn);
    for(int j=0 ; j < knn ;++j){
        double* position = index.getPoint(indices[0][j]);
        out[j] = std::make_pair(Eigen::Vector3d(position[0], position[1], position[2]), sqrt(dists[0][j]));
    }
    return 0;

}


