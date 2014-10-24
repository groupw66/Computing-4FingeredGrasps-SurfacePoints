#pragma once

#include "GlobalCommon.h"
#include "mVect.h"

#define DEFAULT_PYRAMID_SIDE        32
#define GP_DATATYPE double



typedef GP_DATATYPE data6[6];
typedef GP_DATATYPE data3[3];
typedef mvPoint3<GP_DATATYPE> vector3;
typedef mvPoint2<GP_DATATYPE> vector2;
typedef mvQuaternion<GP_DATATYPE> myQuaternion;
typedef mvMatrix44<GP_DATATYPE> myMatrix44;

/**
 * sub-structore for 3D grasping point, representing the torque part of the grasping point
 */
class TorqueFan {
public:
    vector3 v1,v2;      //boundary vector of the fan
    vector3 normal;     //normal vector of the fan, when the fan is a plane
    bool isPlane;
    bool crossPlane(vector3 plane);         //return true iff [plane] is not on the line containing normal
    bool crossPlane(TorqueFan fan);         //return true [fan] has non-boundary intersection with the plane
};

/**
 * sub-structure for 2D grasping point, definind a plane intersected with a plane
 *
 */
class IntersectedFan {
public:
    vector2 p1,p2;                // position of the projected
    bool hasSeg;                    // if true, the projected is a segment between p1 and p2
    bool segIsPositive;             // valid when isSeg is true only
    bool hasPosRay;                 // the projected fan consists of a positive ray, from
    bool hasNegRay;                 // the projected fan consists of a positive ray
    vector2 posRayPoint;
    vector2 negRayPoint;
    vector2 posRayDirection;
    vector2 negRayDirection;
    vector2 zeroRay[2];
    int zeroRayCount;
};

/**
 * class representing 3D point contact with friction
 */
class GraspingPoint3D {
public:
    GraspingPoint3D(vector3 position,vector3 normal,double halfAngleDegree = 10.d);
    GraspingPoint3D();
    ~GraspingPoint3D();

    int idx;

	unsigned long userData;			    // index of contact point
    TorqueFan torqueFan;                // torque fan of the

    //getter
    const vector3& getNormal();
    const vector3& getPosition();
    data6* getWrench(int idx);
    data6* getNormalWrench();      // get the wrench of the normal vector (for frictionelss contact)
    int getWrenchCount();
    void getG(double **dest,int offset);    //copy G (transformation matrix to dest, starting with the column [offset]
    double getHalfAngle();

    //sub-structore construction
    void genNormalWrench(GP_DATATYPE ox = 0,GP_DATATYPE oy = 0,GP_DATATYPE oz = 0);
    void genWrenches(GP_DATATYPE ox = 0,GP_DATATYPE oy = 0,GP_DATATYPE oz = 0,int numWrenches = DEFAULT_PYRAMID_SIDE);
    void genTorqueFan(GP_DATATYPE ox = 0,GP_DATATYPE oy = 0,GP_DATATYPE oz = 0);
    void genLocalFrame();                   //generate the local frame and G of this contact point

    void clearWrench();

    //for debug
    void printForceConeGL(FILE* fp = stdout);
    void printTorqueConeGL(FILE* fp = stdout);
    void printTorqueFanGL(FILE* fp = stdout);


private:
    double halfangle;                       // half angle (in radian)
    double sinHalfangle;                    // half angle (in radian)
    double cosHalfangle;                    // half angle (in radian)
    double tanHalfangle;

    vector3 position;                       // position vector
    vector3 normal;                         // unit normal vector

    int numWrenches;
    data6* wrenches;                        // wrenches of this graspin point;
    data6* normalWrench;

	//local contact frame
	vector3 tangent1,tangent2;              //axes of the local contact frame (normal = normal direction, tangent1 x tangent2 = norm)
	GP_DATATYPE** G;                        //transformation matrix, W = Gx 	(W is a wrench, x is the intensity of each axes

};

class GraspingPoint2D {
public:
    GraspingPoint2D(vector2 position,vector2 normal,double halfAngle);
    GraspingPoint2D();
    ~GraspingPoint2D();

	unsigned long userData;			    // index of contact point
    IntersectedFan fan;

    //getter
    vector2 getNormal();
    vector2 getPosition();
    vector3* getWrench(int idx);


    void genFan(GP_DATATYPE ox = 0,GP_DATATYPE oy = 0);  //generate the intersecton of the wrench on z=1 plane
    void genWrenches(GP_DATATYPE ox = 0,GP_DATATYPE oy = 0);   //generate the wrench of the grasping point

private:
    double halfangle;                       // half angle (in radian)
    double sinHalfangle;                    // half angle (in radian)
    double cosHalfangle;                    // half angle (in radian)
    double angle;                           // angle to the +x-axis (in radian)

    vector2 position;                     // position vector
    vector2 normal;                       // unit normal vector
    vector2 leftForce,rightForce;         // left and right boundary force vectors
    vector3 leftWrench,rightWrench;       // left and right wrenches
};



//base class for Grasp Tester
class BaseGraspTester {
public:
    //for frictional four finger
    virtual bool isForceClosure(GraspingPoint3D &gp1,GraspingPoint3D &gp2,GraspingPoint3D &gp3,GraspingPoint3D &gp4) = 0;
    //for frictionless seven finger
    virtual bool isForceClosure(
        GraspingPoint3D &gp1,
        GraspingPoint3D &gp2,
        GraspingPoint3D &gp3,
        GraspingPoint3D &gp4,
        GraspingPoint3D &gp5,
        GraspingPoint3D &gp6,
        GraspingPoint3D &gp7
        ) {
        printf("frictionless NOT SUPPORTED\n");
        return false;
    };

    //for frictional n finger
    //virtual bool isForceClosure(int numGP,GraspingPoint3D** gpList) = 0;
};
