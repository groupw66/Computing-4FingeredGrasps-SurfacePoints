#include <UnitTest++.h>
#include <Eigen/Dense>
#include <cmath>
#include "Compute4FingeredGrasps.h"

namespace
{

TEST(isPointInCone_true0)
{
    double angle = 10.0/180.0*M_PI;
    Eigen::Vector3d point(1,0,0);
    Eigen::Vector3d normal(1,-1,0);
    Eigen::Vector3d position(0,1,0);
    const bool Result = Compute4FingeredGrasps::isPointInCone(point, SurfacePoint(normal, position), angle);
    CHECK_EQUAL(true, Result);
}

TEST(isPointInCone_true1)
{
    double angle = 10.0/180.0*M_PI;
    Eigen::Vector3d point(1,0,0);
    Eigen::Vector3d normal(1.000005,-1.00005,0);
    Eigen::Vector3d position(0,1,0);
    const bool Result = Compute4FingeredGrasps::isPointInCone(point, SurfacePoint(normal, position), angle);
    CHECK_EQUAL(true, Result);
}

TEST(isPointInCone_false0)
{
    double angle = 10.0/180.0*M_PI;
    Eigen::Vector3d point(1,0,0);
    Eigen::Vector3d normal(1.500005,-1.00005,0);
    Eigen::Vector3d position(0,1,0);
    const bool Result = Compute4FingeredGrasps::isPointInCone(point, SurfacePoint(normal, position), angle);
    CHECK_EQUAL(false, Result);
}

TEST(isPointInConeDoubleside_true0)
{
    double angle = 10.0/180.0*M_PI;
    Eigen::Vector3d point(1,0,0);
    Eigen::Vector3d normal(1.000005,-1.00005,0);
    Eigen::Vector3d position(0,1,0);
    const bool Result = Compute4FingeredGrasps::isPointInConeDoubleside(point, SurfacePoint(normal, position), angle);
    CHECK_EQUAL(true, Result);
}

TEST(isPointInConeDoubleside_true1)
{
    double angle = 10.0/180.0*M_PI;
    Eigen::Vector3d point(0,1,0);
    Eigen::Vector3d normal(-1.000005,1.00005,0);
    Eigen::Vector3d position(1,0,0);
    const bool Result = Compute4FingeredGrasps::isPointInConeDoubleside(point, SurfacePoint(normal, position), angle);
    CHECK_EQUAL(true, Result);
}

TEST(isPointInConeDoubleside_false0)
{
    double angle = 10.0/180.0*M_PI;
    Eigen::Vector3d point(1,0,0);
    Eigen::Vector3d normal(1.500005,-1.00005,0);
    Eigen::Vector3d position(0,1,0);
    const bool Result = Compute4FingeredGrasps::isPointInConeDoubleside(point, SurfacePoint(normal, position), angle);
    CHECK_EQUAL(false, Result);
}



}
