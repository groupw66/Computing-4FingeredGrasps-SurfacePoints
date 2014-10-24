#ifndef __DAE_INTERSECTING_H_INCLUDED__
#define __DAE_INTERSECTING_H_INCLUDED__

#include <assert.h>
#include "GraspingPoint.h"
#include "daeUtils.h"

typedef struct ExtendedPoint_TAG {
    bool isPai;     // is a point-at-infinity ?
    vector2 point;

    ExtendedPoint_TAG(bool aisPai,vector2 apoint) {
        isPai = aisPai;
        point = apoint;
    }

    ExtendedPoint_TAG() : isPai(0), point(0,0) { }
} ExtendedPoint;

typedef SmallStore<ExtendedPoint> ConvexHull;

class DaeWrenchFanChecker {
public:
    DaeWrenchFanChecker();
    ~DaeWrenchFanChecker();
    bool isForceClosure(GraspingPoint2D& gp1,GraspingPoint2D& gp2,GraspingPoint2D &gp3);
private:
    static float crossExtendedPoint(const ExtendedPoint& p1,const ExtendedPoint& p2,const ExtendedPoint& p3);
    bool SegInConvexHull(const ExtendedPoint &c1,const ExtendedPoint &c2,
                         const ExtendedPoint &c3,const ExtendedPoint &c4,
                         const ExtendedPoint &p1,const ExtendedPoint &p2);
    void genConvexHullExtended(const ExtendedPoint &p1,const ExtendedPoint &p2,
                               const ExtendedPoint &p3,const ExtendedPoint &p4,
                               ConvexHull &cHull);
};



#endif