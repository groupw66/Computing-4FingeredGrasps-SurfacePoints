#include <UnitTest++.h>
#include <iostream>
#include <Eigen/Dense>
#include "Grasp.h"

namespace
{

TEST(Grasp_ctor1_0)
{
    Grasp g(1, 20, 12, 15);
    CHECK_EQUAL(4,g.surfacePoints.size());
}

TEST(Grasp_get)
{
    Grasp g(1, 20, 12, 15);
    CHECK_EQUAL(1,g[0]);
    CHECK_EQUAL(12,g[1]);
    CHECK_EQUAL(15,g[2]);
    CHECK_EQUAL(20,g[3]);
}

}
