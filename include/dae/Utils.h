#ifndef __GRASPING_UTILS_INCLUDED__
#define __GRASPING_UTILS_INCLUDED__

#include "mVect.h"

/**
 * Given v1 and v2, return a unit vector on the plane containing v1 and v2 
 *   that is perpendicular to v1 and is on the same side as v2, with respect to 
 *   the line containing v1
 */
mvPoint3d GramSchmidt(const mvPoint3d& v1,const mvPoint3d& v2,bool normalize = true);

/**
 * Checking which side of the plane containing v1 and v2 that v3 lies on
 */
float vectorSide(const mvPoint3d &v1,const mvPoint3d &v2,const mvPoint3d &v3);

/**
 * Generate 3x3 rotational matrix that rotate around a unit vector [v]. The angle of rotation r
 * is describe with [cr] = cos(r) and [sr] = sin(r)
 */
void genRot(const mvPoint3d &v,mvMatrix44d &r,double cr,double sr);

#endif