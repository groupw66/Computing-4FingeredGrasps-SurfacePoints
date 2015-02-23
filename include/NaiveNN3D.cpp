#include "NaiveNN3D.h"

NaiveNN3D::NaiveNN3D()
{
    //ctor
}

NaiveNN3D::~NaiveNN3D()
{
    //dtor
}

NaiveNN3D::NaiveNN3D(const std::vector<Eigen::Vector3d> &points)
{
    initPoints(points);
}

int NaiveNN3D::initPoints(const std::vector<Eigen::Vector3d> &points)
{
    dataset = points;
    return 0;
}

template <class T1, class T2, class Pred = std::less<T2> >
struct sort_pair_second {
    bool operator()(const std::pair<T1,T2>&left, const std::pair<T1,T2>&right) {
        Pred p;
        return p(left.second, right.second);
    }
};

int NaiveNN3D::knnSearch(const std::vector<Eigen::Vector3d>& points, std::vector< std::vector< std::pair<Eigen::Vector3d, double> > >& out, int knn)
{
    out.resize(points.size());
    for(unsigned int i=0 ; i < points.size() ; ++i){
        out[i].resize(knn);
        Eigen::Vector3d p = points[i];
        for(int j=0 ; j < knn ;++j){
            out[i][j] = std::make_pair(Eigen::Vector3d(),  std::numeric_limits<double>::max());
        }
        for(unsigned int j=0 ; j < dataset.size() ;++j){
            double dist = (dataset[j]-p).norm();
            if(dist < out[i][knn-1].second){
                out[i][knn-1] = std::make_pair(dataset[j], dist);
                std::sort(out[i].begin(), out[i].end(), sort_pair_second<Eigen::Vector3d, double>());
            }
        }
    }
    return 0;
}
