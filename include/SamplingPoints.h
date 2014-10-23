#ifndef SAMPLINGPOINTS_H
#define SAMPLINGPOINTS_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>

namespace SamplingPoints
{
    std::vector<Eigen::Vector3d> uniformAxis(Eigen::Vector3d minAABB, Eigen::Vector3d maxAABB, int npointPerAxis);
    std::vector<Eigen::Vector3d> uniform(Eigen::Vector3d minAABB, Eigen::Vector3d maxAABB, int npoint);
    std::vector<Eigen::Vector3d> randomUniform(Eigen::Vector3d minAABB, Eigen::Vector3d maxAABB, int npoint);
    std::vector<Eigen::Vector3d> randomNormalDist(Eigen::Vector3d mean, Eigen::Vector3d sd, int npoint);
    std::vector<Eigen::Vector3d> randomNormalDist(Eigen::Vector3d mean, Eigen::Vector3d sd, int npoint, Eigen::Vector3d minAABB, Eigen::Vector3d maxAABB);
};

#endif // SAMPLINGPOINTS_H
