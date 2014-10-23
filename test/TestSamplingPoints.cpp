#include <UnitTest++.h>
#include <Eigen/Dense>
#include <vector>
#include "SamplingPoints.h"

namespace
{

TEST(SamplingPoints_uniformAxis)
{
    Eigen::Vector3d minAABB(-1000.5d, -500.6d, -200.3d);
    Eigen::Vector3d maxAABB(555.5d, 100.2d, 333.3d);
    int npointsPerAxis = 50;
    std::vector<Eigen::Vector3d> points = SamplingPoints::uniformAxis(minAABB, maxAABB, npointsPerAxis);
    CHECK_EQUAL(npointsPerAxis*npointsPerAxis*npointsPerAxis, points.size());
}

TEST(SamplingPoints_uniform)
{
    Eigen::Vector3d minAABB(-1000.5d, -500.6d, -200.3d);
    Eigen::Vector3d maxAABB(555.5d, 100.2d, 333.3d);
    int npoints = 125000;
    std::vector<Eigen::Vector3d> points = SamplingPoints::uniform(minAABB, maxAABB, npoints);
    CHECK_CLOSE(npoints, points.size(), npoints*0.2);
}

TEST(SamplingPoints_randomUniform)
{
    Eigen::Vector3d minAABB(-1000.5d, -500.6d, -200.3d);
    Eigen::Vector3d maxAABB(555.5d, 100.2d, 333.3d);
    int npoints = 125000;
    std::vector<Eigen::Vector3d> points = SamplingPoints::randomUniform(minAABB, maxAABB, npoints);
    CHECK_EQUAL(npoints, points.size());
}

TEST(SamplingPoints_randomNormalDist)
{
    Eigen::Vector3d mean(0,0,0);
    Eigen::Vector3d sd(10,20,30);
    int npoints = 125000;
    std::vector<Eigen::Vector3d> points = SamplingPoints::randomNormalDist(mean, sd, npoints);
    CHECK_EQUAL(npoints, points.size());
}

TEST(SamplingPoints_randomNormalDist2)
{
    Eigen::Vector3d mean(0,0,0);
    Eigen::Vector3d sd(10,20,30);
    Eigen::Vector3d minAABB(-1000.5d, -500.6d, -200.3d);
    Eigen::Vector3d maxAABB(555.5d, 100.2d, 333.3d);
    int npoints = 125000;
    std::vector<Eigen::Vector3d> points = SamplingPoints::randomNormalDist(mean, sd, npoints, minAABB, maxAABB);
    CHECK_EQUAL(npoints, points.size());
}

}
