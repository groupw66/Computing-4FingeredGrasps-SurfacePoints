#include <UnitTest++.h>
#include <iostream>
#include <Eigen/Dense>
#include "SurfacePoint.h"
#include "BasicGeometry.hpp"

namespace
{

TEST(SurfacePoint_ctor)
{
    Eigen::Vector3d position(1.5, 20, 398.312421);
    Eigen::Vector3d normal(1, -1, 5);
    SurfacePoint surfacePoint(position,normal);
    CHECK_EQUAL(position, surfacePoint.position);
    CHECK_EQUAL(normal.normalized(), surfacePoint.normal);
}

TEST(SurfacePoint_assign)
{
    Eigen::Vector3d position(1.5, 20, 398.312421);
    Eigen::Vector3d normal(1, -1, 5);
    SurfacePoint surfacePoint;
    SurfacePoint surfacePoint2(position,normal);
    surfacePoint = surfacePoint2;
    CHECK_EQUAL(position, surfacePoint.position);
    CHECK_EQUAL(normal.normalized(), surfacePoint.normal);
}

TEST(SurfacePoint_lessthan_true)
{
    SurfacePoint surfacePoint1(Eigen::Vector3d(0.d,1.d,2.d), Eigen::Vector3d(3.d,4.d,5.d));
    SurfacePoint surfacePoint2(Eigen::Vector3d(0.d,2.d,2.d), Eigen::Vector3d(3.d,4.d,5.d));
    CHECK_EQUAL(true, surfacePoint1 < surfacePoint2);
}

TEST(SurfacePoint_lessthan_false0)
{
    SurfacePoint surfacePoint1(Eigen::Vector3d(0.d,1.d,2.d), Eigen::Vector3d(3.d,4.d,5.d));
    SurfacePoint surfacePoint2(Eigen::Vector3d(0.d,1.d,2.d), Eigen::Vector3d(1.d,4.d,5.d));
    CHECK_EQUAL(false, surfacePoint1 < surfacePoint2);
}

TEST(SurfacePoint_lessthan_false1)
{
    SurfacePoint surfacePoint1(Eigen::Vector3d(0.d,1.d,2.d), Eigen::Vector3d(3.d,4.d,5.d));
    SurfacePoint surfacePoint2(Eigen::Vector3d(0.d,1.d,2.d), Eigen::Vector3d(3.d,4.d,5.d));
    CHECK_EQUAL(false, surfacePoint1 < surfacePoint2);
}

TEST(SurfacePoint_getFrictionCone)
{
    SurfacePoint surfacePoint1(Eigen::Vector3d(0.d,1.d,2.d), Eigen::Vector3d(3.d,4.d,5.d));
    double halfAngle = 10.d;
    int nPyramidSide = 8;
    std::vector<Eigen::Vector3d> frictionCone = surfacePoint1.getFrictionCone(halfAngle, nPyramidSide);
    CHECK_EQUAL(8, frictionCone.size());
    for(auto f : frictionCone){
        CHECK_CLOSE(halfAngle*M_PI/180.d, Geometry::angleBetweenVectors(f,surfacePoint1.normal), 1e-6);
    }
}

TEST(SurfacePoint_getWrenchCone)
{
    SurfacePoint surfacePoint1(Eigen::Vector3d(0.d,1.d,2.d), Eigen::Vector3d(3.d,4.d,5.d));
    Eigen::Vector3d cm(0,0,0);
    double halfAngle = 10.d;
    int nPyramidSide = 8;
    std::vector<Wrench> wrenchCone = surfacePoint1.getWrenchCone(cm, halfAngle, nPyramidSide);
    CHECK_EQUAL(8, wrenchCone.size());
    for(auto wrench : wrenchCone){
        CHECK_CLOSE(halfAngle*M_PI/180.d, Geometry::angleBetweenVectors(wrench.force(),surfacePoint1.normal), 1e-6);
        //TO-DO
    }
}

}
