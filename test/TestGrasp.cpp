#include <UnitTest++.h>
#include <iostream>
#include <Eigen/Dense>
#include "Grasp.h"

namespace
{

TEST(Grasp_ctor1_0)
{
    SurfacePoint sp1(Eigen::Vector3d(1,2,3), Eigen::Vector3d(4,5,6));
    SurfacePoint sp2(Eigen::Vector3d(4,5,6), Eigen::Vector3d(7,8,9));
    SurfacePoint sp3(Eigen::Vector3d(7,8,9), Eigen::Vector3d(0,1,2));
    SurfacePoint sp4(Eigen::Vector3d(0,1,2), Eigen::Vector3d(3,4,5));
    Grasp g(sp1, sp2, sp3, sp4);
    CHECK_EQUAL(4,g.surfacePoints.size());
}

/* //duplicate point not allowed
TEST(Grasp_ctor1_1)
{
    SurfacePoint sp1(Eigen::Vector3d(1,2,3), Eigen::Vector3d(4,5,6));
    SurfacePoint sp2(Eigen::Vector3d(4,5,6), Eigen::Vector3d(7,8,9));
    SurfacePoint sp3(Eigen::Vector3d(7,8,9), Eigen::Vector3d(0,1,2));
    SurfacePoint sp4(Eigen::Vector3d(7,8,9), Eigen::Vector3d(0,1,2));
    Grasp g(sp1, sp2, sp3, sp4);
    CHECK_EQUAL(3,g.surfacePoints.size());
}
*/

TEST(Grasp_ctor1_2)
{
    SurfacePoint sp1(Eigen::Vector3d(1,2,3), Eigen::Vector3d(4,5,6));
    SurfacePoint sp2(Eigen::Vector3d(4,5,6), Eigen::Vector3d(7,8,9));
    SurfacePoint sp3(Eigen::Vector3d(7,8,9), Eigen::Vector3d(0,1,2));
    SurfacePoint sp4(Eigen::Vector3d(0,1,2), Eigen::Vector3d(3,4,5));
    Grasp g(sp1, sp2, sp3, sp4);
    CHECK_EQUAL(4,g.surfacePoints.size());
}

//TEST(Grasp_ctor2...)

}
