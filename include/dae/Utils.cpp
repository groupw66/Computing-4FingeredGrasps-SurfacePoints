#include "Utils.h"

void genRot(const mvPoint3d &v,mvMatrix44d &r,double cr,double sr)
{
    double x = v.x;
    double y = v.y;
    double z = v.z;

    r(0,0) = cr + (1 - cr) * x * x;     
    r(0,1) = (1 - cr) * x * y - sr * z;
    r(0,2) = (1 - cr) * x * z + sr * y;

    r(1,0) = (1 - cr) * y * x + sr * z;
    r(1,1) = cr + (1 - cr) * y * y;     
    r(1,2) = (1 - cr) * y * z - sr * x;

    r(2,0) = (1 - cr) * z * x - sr * y;
    r(2,1) = (1 - cr) * z * y + sr * x;
    r(2,2) = cr + (1 - cr) * z * z;     
}

mvPoint3d GramSchmidt(const mvPoint3d& v1,const mvPoint3d& v2,bool normalize)
{
    double dot = v1 ^ v2;

    mvPoint3d v2_on_v1 = v1 * dot;
    mvPoint3d newv2 = v2 - v2_on_v1;
    if (normalize) {
        return newv2.normalize();
    } else {
        return newv2;
    }
}
