#include <UnitTest++.h>
#include <iostream>
#include <Eigen/Dense>
#include "SurfacePoint.h"

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

TEST(SurfacePoint_lessthan)
{
    SurfacePoint surfacePoint1(Eigen::Vector3d(0.d,1.d,2.d), Eigen::Vector3d(3.d,4.d,5.d));
    SurfacePoint surfacePoint2(Eigen::Vector3d(0.d,2.d,2.d), Eigen::Vector3d(3.d,4.d,5.d));
    CHECK_EQUAL(true, surfacePoint1 < surfacePoint2);
    CHECK_EQUAL(true, surfacePoint1 <= surfacePoint2);
    CHECK_EQUAL(false, surfacePoint1 == surfacePoint2);
    CHECK_EQUAL(false, surfacePoint1 >= surfacePoint2);
    CHECK_EQUAL(false, surfacePoint1 > surfacePoint2);
    CHECK_EQUAL(true, surfacePoint1 != surfacePoint2);
}

TEST(SurfacePoint_equal)
{
    SurfacePoint surfacePoint1(Eigen::Vector3d(0.d,1.d,2.d), Eigen::Vector3d(3.d,4.d,5.d));
    SurfacePoint surfacePoint2(Eigen::Vector3d(0.d,1.d,2.d), Eigen::Vector3d(3.d,4.d,5.d));
    CHECK_EQUAL(false, surfacePoint1 < surfacePoint2);
    CHECK_EQUAL(true, surfacePoint1 <= surfacePoint2);
    CHECK_EQUAL(true, surfacePoint1 == surfacePoint2);
    CHECK_EQUAL(true, surfacePoint1 >= surfacePoint2);
    CHECK_EQUAL(false, surfacePoint1 > surfacePoint2);
    CHECK_EQUAL(false, surfacePoint1 != surfacePoint2);
}

TEST(SurfacePoint_greaterthan)
{
    SurfacePoint surfacePoint1(Eigen::Vector3d(0.d,1.d,2.d), Eigen::Vector3d(30.d,4.d,5.d));
    SurfacePoint surfacePoint2(Eigen::Vector3d(0.d,1.d,2.d), Eigen::Vector3d(3.d,4.d,5.d));
    CHECK_EQUAL(false, surfacePoint1 < surfacePoint2);
    CHECK_EQUAL(false, surfacePoint1 <= surfacePoint2);
    CHECK_EQUAL(false, surfacePoint1 == surfacePoint2);
    CHECK_EQUAL(true, surfacePoint1 >= surfacePoint2);
    CHECK_EQUAL(true, surfacePoint1 > surfacePoint2);
    CHECK_EQUAL(true, surfacePoint1 != surfacePoint2);
}

}
