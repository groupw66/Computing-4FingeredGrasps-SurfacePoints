#include <UnitTest++.h>
#include <Eigen/Dense>
#include <cmath>
#include "BasicGeometry.hpp"

namespace
{
//////////////////////////////
//isvectorsPositivelySpan3D
////////////////////////////////
//4P

TEST(isVectorsPositivelySpan3D_4P_true0)
{
    std::vector<Eigen::Vector3d> vectors;
    vectors.push_back(Eigen::Vector3d(1,0,0));
    vectors.push_back(Eigen::Vector3d(0,1,0));
    vectors.push_back(Eigen::Vector3d(0,0,1));
    vectors.push_back(Eigen::Vector3d(-1,-1,-1));
    const bool Result = Geometry::isVectorsPositivelySpan3D(vectors);
    const bool Result2 = Geometry::isVectorsPositivelySpan3DQhull(vectors);
    CHECK_EQUAL(true, Result);
    CHECK_EQUAL(Result, Result2);
}

TEST(isVectorsPositivelySpan3D_4P_false0)
{
    std::vector<Eigen::Vector3d> vectors;
    vectors.push_back(Eigen::Vector3d(1,0,0));
    vectors.push_back(Eigen::Vector3d(0,1,0));
    vectors.push_back(Eigen::Vector3d(0,0,1));
    vectors.push_back(Eigen::Vector3d(1,1,1));
    const bool Result = Geometry::isVectorsPositivelySpan3D(vectors);
    const bool Result2 = Geometry::isVectorsPositivelySpan3DQhull(vectors);
    CHECK_EQUAL(false, Result);
    CHECK_EQUAL(Result, Result2);
}
/*
TEST(isVectorsPositivelySpan3D_4P_false_plane)
{
}
*/

}
