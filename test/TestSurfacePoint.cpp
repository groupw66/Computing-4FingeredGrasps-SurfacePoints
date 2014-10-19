#include <UnitTest++.h>
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

}
