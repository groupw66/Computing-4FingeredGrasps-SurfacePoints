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

}
