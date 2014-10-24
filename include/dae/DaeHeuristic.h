#ifndef __DAE_HEURISTIC_H_INCLUDED__
#define __DAE_HEURISTIC_H_INCLUDED__

#include <assert.h>
#include "GraspingPoint.h"
#include "SurfacePoint.h"
//#include "daeUtils.h"
//#include "YHL.h"
//#include "gjk_wrapper.h"
//#include "qhull_wrapper.h"
//#include "QDistance.h"

/*
 * Dual points (taken from gjkTest2 project)
 */

class ReducedPoint {
public:
    mvPoint2<GP_DATATYPE> p;
    int colorTag; // 0 = grey, -1 = red, 1 = blue; the sign is the same as the torque
    double angle;
};

typedef ReducedPoint *ReducedPointPtr;


class DaeHeuristicChecker : public BaseGraspTester{
public:
    DaeHeuristicChecker(double halfAngleRad,int maxWrench = 1024);
    ~DaeHeuristicChecker();

	void setHalfAngle(double halfAngleRad);

	// This function is heuristic (possible to return false positive, but no false negative)
    bool isForceClosure(GraspingPoint3D &gp1,GraspingPoint3D &gp2,GraspingPoint3D &gp3,GraspingPoint3D &gp4);

    inline static vector3 getVector3(Eigen::Vector3d v){
        return vector3(v.x(), v.y(), v.z());
    }

    inline static GraspingPoint3D getGraspingPoint3D(SurfacePoint sp){
        return GraspingPoint3D(getVector3(sp.position), getVector3(sp.normal.normalized()), 10.d);
    }

    inline bool isForceClosure(SurfacePoint &sp1,SurfacePoint &sp2,SurfacePoint &sp3,SurfacePoint &sp4){
        GraspingPoint3D gp1 = getGraspingPoint3D(sp1);
        GraspingPoint3D gp2 = getGraspingPoint3D(sp2);
        GraspingPoint3D gp3 = getGraspingPoint3D(sp3);
        GraspingPoint3D gp4 = getGraspingPoint3D(sp4);
        return  this->isForceClosure(gp1, gp2, gp3, gp4);
    }

    //bool isForceClosure(int numGP,GraspingPoint3D** gpList);

    // for 3d frictionless seven finger (heuristic)
    /*
    bool isForceClosure(
        GraspingPoint3D &gp1,
        GraspingPoint3D &gp2,
        GraspingPoint3D &gp3,
        GraspingPoint3D &gp4,
        GraspingPoint3D &gp5,
        GraspingPoint3D &gp6,
        GraspingPoint3D &gp7
    );
    */


    void test();

	bool isPosSpan6vect(const mvPoint3d &v1,const mvPoint3d &v2,
						const mvPoint3d &v3,const mvPoint3d &v4,
						const mvPoint3d &v5,const mvPoint3d &v6);


private:
    /*
    YHLPosSpanTest *yhl;
    GJKPosSpanTest *gjk;
    QDistPosSpanTest *qdist;
    QHPosSpanTest *qhull;
    */
    GP_DATATYPE **wrenches;
    vector3 *vect;

	double sinHalfAngle,cosHalfAngle,halfAngle,fullAngle;
    double sinFullAngle,cosFullAngle;
    double sinMinusHalfAngle,cosMinusHalfAngle;
    double sinMinusFullAngle,cosMinusFullAngle;

	bool isPosSpan4vect(const mvPoint3d &v1,const mvPoint3d &v2,const mvPoint3d &v3,const mvPoint3d &v4);

    //bool isPosSpanNVect(int numVect,vector3 *vect);


    /*
     For isForceConeSpanX, the input vectors must be normalized
     */
	bool isForceConeSpan2(const mvPoint3d &v1,const mvPoint3d &v2);
	bool isForceConeSpan3(const mvPoint3d &v1,const mvPoint3d &v2,const mvPoint3d &v3);
	bool isForceConeSpan4(const mvPoint3d &v1,const mvPoint3d &v2,const mvPoint3d &v3,const mvPoint3d &v4);
/*
    bool isForceSpan4ByYHL(GraspingPoint3D &gp1,GraspingPoint3D &gp2,GraspingPoint3D &gp3,GraspingPoint3D &gp4);
    bool isForceSpan3ByYHL(GraspingPoint3D &gp1,GraspingPoint3D &gp2,GraspingPoint3D &gp3);
    bool isForceSpan2ByYHL(GraspingPoint3D &gp1,GraspingPoint3D &gp2);

    bool isForceSpan4ByGJK(GraspingPoint3D &gp1,GraspingPoint3D &gp2,GraspingPoint3D &gp3,GraspingPoint3D &gp4);
    bool isForceSpan3ByGJK(GraspingPoint3D &gp1,GraspingPoint3D &gp2,GraspingPoint3D &gp3);
    bool isForceSpan2ByGJK(GraspingPoint3D &gp1,GraspingPoint3D &gp2);

    bool isForceConeSpan2Test(GraspingPoint3D &gp1,GraspingPoint3D &gp2);
    bool isForceConeSpan3Test(GraspingPoint3D &gp1,GraspingPoint3D &gp2,GraspingPoint3D &gp3);
    bool isForceConeSpan4Test(GraspingPoint3D &gp1,GraspingPoint3D &gp2,GraspingPoint3D &gp3,GraspingPoint3D &gp4);
*/
    // is -v3 lies in the middle zone of v1 and v2
    bool isInMiddleZone(const mvPoint3d &v1,const mvPoint3d &v2,const mvPoint3d &v3);

    bool isTorqueSpan(const mvPoint3d &origin,
                      GraspingPoint3D &gp1,
                      GraspingPoint3D &gp2,
                      GraspingPoint3D &gp3);


    //bool isTorqueSpan(const mvPoint3d &origin,int numGP,GraspingPoint3D** gpList);

    void generateTorqueFan(const mvPoint3d &pos,const mvPoint3d &norm,mvPoint3d &left,mvPoint3d &right);

    /*
     * for frictionless
     */
    bool is3DPosSpan(mvPoint3d *points,int numPoint);
    bool is3DPosSpanBrute(mvPoint3d *points,int numPoint);

    bool checkTorque(
        vector3 newOrigin,
        GraspingPoint3D &gp1,
        GraspingPoint3D &gp2,
        GraspingPoint3D &gp3,
        GraspingPoint3D &gp4,
        GraspingPoint3D &gp5,
        GraspingPoint3D &gp6);

};



#endif
