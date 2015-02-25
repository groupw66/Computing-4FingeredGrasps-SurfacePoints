#include "NaiveNN3D.h"

NaiveNN3D::NaiveNN3D()
{
    //ctor
}

NaiveNN3D::~NaiveNN3D()
{
    //dtor
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

template <class T1, class T2, class T3, class Pred = std::less<T2> >
struct sort_tuple_third {
    bool operator()(const std::tuple<T1,T2,T3>&left, const std::tuple<T1,T2,T3>&right) {
        Pred p;
        return p(std::get<2>(left), std::get<2>(right));
    }
};


int NaiveNN3D::knnSearch(const std::vector<Eigen::Vector3d>& points, std::vector< std::vector< std::tuple<Eigen::Vector3d, int, double> > >& out, int knn)
{
    out.resize(points.size());
    for(unsigned int i=0 ; i < points.size() ; ++i){
        out[i].resize(knn);
        Eigen::Vector3d p = points[i];
        for(int j=0 ; j < knn ;++j){
            out[i][j] = std::make_tuple(Eigen::Vector3d(), -1, std::numeric_limits<double>::max());
        }
        for(unsigned int j=0 ; j < dataset.size() ;++j){
            double dist = (dataset[j]-p).norm();
            if(dist < std::get<2>(out[i][knn-1])){
                out[i][knn-1] = std::make_tuple(dataset[j], j, dist);
                std::sort(out[i].begin(), out[i].end(),
                            [](std::tuple<Eigen::Vector3d, int, double> const &t1, std::tuple<Eigen::Vector3d, int, double> const &t2) {
                            return std::get<2>(t1) < std::get<2>(t2);}
                            );
            }
        }
    }
    return 0;
}

int NaiveNN3D::knnSearch(const Eigen::Vector3d& point, std::vector< std::tuple<Eigen::Vector3d, int, double> >& out, int knn)
{
    out.resize(knn);
    Eigen::Vector3d p = point;
    for(int j=0 ; j < knn ;++j){
        out[j] = std::make_tuple(Eigen::Vector3d(), -1, std::numeric_limits<double>::max());
    }
    for(unsigned int j=0 ; j < dataset.size() ;++j){
        double dist = (dataset[j]-p).norm();
        if(dist < std::get<2>(out[knn-1])){
            out[knn-1] = std::make_tuple(dataset[j], j, dist);
            std::sort(out.begin(), out.end(),
                        [](std::tuple<Eigen::Vector3d, int, double> const &t1, std::tuple<Eigen::Vector3d, int, double> const &t2) {
                        return std::get<2>(t1) < std::get<2>(t2);}
                        );
        }
    }
    return 0;

}

