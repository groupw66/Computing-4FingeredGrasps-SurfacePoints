#include <UnitTest++.h>
#include <iostream>
#include <Eigen/Dense>
#include "Wrench.h"
#include "BasicGeometry.hpp"


namespace
{

TEST(Wrench_ctor0)
{
    Eigen::Vector3d position(1.5, 20, 398.312421);
    Eigen::Vector3d force(1, -1, 5);
    Wrench surfacePoint(position, force);
    force.normalize();
    Eigen::Vector3d torque = position.cross(force);
    CHECK_CLOSE(force.x(), surfacePoint.force().x(), 1e-6);
    CHECK_CLOSE(force.y(), surfacePoint.force().y(), 1e-6);
    CHECK_CLOSE(force.z(), surfacePoint.force().z(), 1e-6);
    CHECK_CLOSE(torque.x(), surfacePoint.torque().x(), 1e-6);
    CHECK_CLOSE(torque.y(), surfacePoint.torque().y(), 1e-6);
    CHECK_CLOSE(torque.z(), surfacePoint.torque().z(), 1e-6);
}

TEST(Wrench_ctor1)
{
    double fx = 1, fy = 2, fz = 3, tx = 4, ty = 5, tz = 6;
    Wrench surfacePoint(fx, fy, fz, tx, ty, tz);
    CHECK_CLOSE(fx, surfacePoint(0), 1e-6);
    CHECK_CLOSE(fy, surfacePoint(1), 1e-6);
    CHECK_CLOSE(fz, surfacePoint(2), 1e-6);
    CHECK_CLOSE(tx, surfacePoint(3), 1e-6);
    CHECK_CLOSE(ty, surfacePoint(4), 1e-6);
    CHECK_CLOSE(tz, surfacePoint(5), 1e-6);
}

}
