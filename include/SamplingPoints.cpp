#include "SamplingPoints.h"

std::vector<Eigen::Vector3d> SamplingPoints::uniformAxis(Eigen::Vector3d minAABB, Eigen::Vector3d maxAABB, int npointPerAxis)
{
    std::vector<Eigen::Vector3d> points;
    Eigen::Vector3d diff = maxAABB-minAABB;
    for(int i=1 ; i<=npointPerAxis ; ++i){
        for(int j=1 ; j<=npointPerAxis ; ++j){
            for(int k=1 ; k<=npointPerAxis ; ++k){
                points.push_back(Eigen::Vector3d(minAABB.x()+diff.x()*i/(npointPerAxis+1),
                                                 minAABB.y()+diff.y()*j/(npointPerAxis+1),
                                                 minAABB.z()+diff.z()*k/(npointPerAxis+1) ) );
            }
        }
    }
    return points;
}

std::vector<Eigen::Vector3d> SamplingPoints::uniform(Eigen::Vector3d minAABB, Eigen::Vector3d maxAABB, int npoint)
{
    std::vector<Eigen::Vector3d> points;
    Eigen::Vector3d diff = maxAABB-minAABB;
    double volumn = diff.x() * diff.y() * diff.z();
    double step = std::pow(volumn/npoint, 1.d/3.d);
    for(double x=minAABB.x()+step/2. ; x<maxAABB.x() ; x+=step){
        for(double y=minAABB.y()+step/2. ; y<maxAABB.y() ; y+=step){
            for(double z=minAABB.z()+step/2. ; z<maxAABB.z() ; z+=step){
                points.push_back(Eigen::Vector3d(x, y, z));
            }
        }
    }
    return points;
}

std::vector<Eigen::Vector3d> SamplingPoints::randomUniform(Eigen::Vector3d minAABB, Eigen::Vector3d maxAABB, int npoint)
{
    std::vector<Eigen::Vector3d> points;
    std::uniform_real_distribution<double> randX(minAABB.x(), maxAABB.x());
    std::uniform_real_distribution<double> randY(minAABB.y(), maxAABB.y());
    std::uniform_real_distribution<double> randZ(minAABB.z(), maxAABB.z());
    std::default_random_engine rng;
    rng.seed(std::random_device{}());
    for(int i=0 ; i<npoint ; ++i){
        points.push_back(Eigen::Vector3d(randX(rng), randY(rng), randZ(rng)));
    }
    return points;
}

std::vector<Eigen::Vector3d> SamplingPoints::randomNormalDist(Eigen::Vector3d mean, Eigen::Vector3d sd, int npoint)
{
    std::vector<Eigen::Vector3d> points;
    std::normal_distribution<double> randX(mean.x(), sd.x());
    std::normal_distribution<double> randY(mean.y(), sd.y());
    std::normal_distribution<double> randZ(mean.z(), sd.z());
    std::default_random_engine rng;
    rng.seed(std::random_device{}());
    for(int i=0 ; i<npoint ; ++i){
        points.push_back(Eigen::Vector3d(randX(rng), randY(rng), randZ(rng)));
    }
    return points;
}



