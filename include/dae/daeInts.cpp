#include "daeInts.h"


#define NULL_ITERATOR       -1



DaeWrenchFanChecker::DaeWrenchFanChecker()
{

}

DaeWrenchFanChecker::~DaeWrenchFanChecker()
{

}

bool DaeWrenchFanChecker::isForceClosure(GraspingPoint2D &gp1,GraspingPoint2D &gp2,GraspingPoint2D &gp3) 
{
    gp1.genFan(gp1.getPosition().x,gp1.getPosition().y);
    gp2.genFan(gp1.getPosition().x,gp1.getPosition().y);
    gp3.genFan(gp1.getPosition().x,gp1.getPosition().y);

    //for reduced case where gp1.fan has wrench as a vector on the z=0 lpane
    if ( (gp2.fan.hasSeg &&  gp2.fan.segIsPositive || gp2.fan.hasPosRay) &&
         (gp3.fan.hasSeg && !gp3.fan.segIsPositive || gp3.fan.hasNegRay) ) {
        ExtendedPoint c1 = ExtendedPoint(1,gp1.fan.zeroRay[0]);
        ExtendedPoint c2 = ExtendedPoint(1,gp1.fan.zeroRay[1]);
        ExtendedPoint c3 = ExtendedPoint( (gp2.fan.hasSeg) ? 0 : 0,  (gp2.fan.hasSeg) ? gp2.fan.p1 : gp2.fan.posRayPoint);
        ExtendedPoint c4 = ExtendedPoint( (gp2.fan.hasSeg) ? 0 : 1,  (gp2.fan.hasSeg) ? gp2.fan.p2 : gp2.fan.posRayDirection);

        ExtendedPoint p1 = ExtendedPoint( (gp3.fan.hasSeg) ? 0 : 0,  (gp3.fan.hasSeg) ? gp3.fan.p1 : gp3.fan.negRayPoint);
        ExtendedPoint p2 = ExtendedPoint( (gp3.fan.hasSeg) ? 0 : 1,  (gp3.fan.hasSeg) ? gp3.fan.p2 : gp3.fan.negRayDirection);

        if (SegInConvexHull(c1,c2,c3,c4,p1,p2)) {
            return true;
        } else
            ; // return false;
    }


    if ( (gp2.fan.hasSeg && !gp2.fan.segIsPositive || gp2.fan.hasNegRay) &&
         (gp3.fan.hasSeg &&  gp3.fan.segIsPositive || gp3.fan.hasPosRay) ) {
        ExtendedPoint c1 = ExtendedPoint(1,gp1.fan.zeroRay[0]);
        ExtendedPoint c2 = ExtendedPoint(1,gp1.fan.zeroRay[1]);
        ExtendedPoint c3 = ExtendedPoint( (gp3.fan.hasSeg) ? 0 : 0,  (gp3.fan.hasSeg) ? gp3.fan.p1 : gp3.fan.posRayPoint);
        ExtendedPoint c4 = ExtendedPoint( (gp3.fan.hasSeg) ? 0 : 1,  (gp3.fan.hasSeg) ? gp3.fan.p2 : gp3.fan.posRayDirection);

        ExtendedPoint p1 = ExtendedPoint( (gp2.fan.hasSeg) ? 0 : 0,  (gp2.fan.hasSeg) ? gp2.fan.p1 : gp2.fan.negRayPoint);
        ExtendedPoint p2 = ExtendedPoint( (gp2.fan.hasSeg) ? 0 : 1,  (gp2.fan.hasSeg) ? gp2.fan.p2 : gp2.fan.negRayDirection);

        if (SegInConvexHull(c1,c2,c3,c4,p1,p2)) {
            return true;
        } else
             ; //return false;
    }
          
	return false;
}

bool DaeWrenchFanChecker::SegInConvexHull(const ExtendedPoint &c1,const ExtendedPoint &c2,
                     const ExtendedPoint &c3,const ExtendedPoint &c4,
                     const ExtendedPoint &p1,const ExtendedPoint &p2)
//Construct a convex hull from c1,c2,c3,c4
//  after that check whether the segment p1p2 intersects with the convex hull
//  return the intersection
{
    // find convex hull
    ConvexHull cHull;
    genConvexHullExtended(c1,c2,c3,c4,cHull);

    //   check s3 dae method (stupid...)
    int a,b,c,d,rev,fow,deleter,nextDeleter;
    a = cHull.first();
    if (a != NULL_ITERATOR)
      b = cHull.succ(a);

    /*
    bool inside = true;
    float chValueLeft,chValueRight;
    float segValueNew,segValueOld;

    if (a != NULL_ITERATOR)
      segValueNew = crossExtendedPoint(p1,p2,cHull[a]);
    while (a != NULL_ITERATOR) {

        chValueLeft = crossExtendedPoint(cHull[a],cHull[b],p1);
        chValueRight = crossExtendedPoint(cHull[a],cHull[b],p2);

        segValueOld = segValueNew;
        segValueNew =  crossExtendedPoint(p1,p2,cHull[b]);

        if (chValueLeft * chValueRight < 0 && segValueNew * segValueOld < 0) return true;

        if (chValueLeft <= 0) inside = false;

        a = cHull.succ(a);
        b = cHull.cyclic_succ(b);

    } 

    return inside; 

    */

    bool inside;
    while (a != NULL_ITERATOR) {
    
        if (crossExtendedPoint(cHull[a],cHull[b],p1) < 0) {
            //reverse (start from a)
            rev = cHull.cyclic_pred(a);
            while (crossExtendedPoint(cHull[rev],cHull[a],p1) < 0) {
                a = rev;
                rev = cHull.cyclic_pred(rev);
            } 

            //forward (start from b)
            fow = cHull.cyclic_succ(b);
            while (crossExtendedPoint(p1,cHull[b],cHull[fow]) < 0) {
                b = fow;
                fow = cHull.cyclic_succ(fow);
            }
            inside = false;

            //outside cone case (s3 definitely does not intersect with the Convex 
            if (crossExtendedPoint(p1,cHull[a],p2) > 0) return false;
            if (crossExtendedPoint(p1,cHull[b],p2) < 0) return false;

            if (b == cHull.cyclic_succ(a)) {
                //inside cone but not inside convex hull (3-point case)
                if (crossExtendedPoint(cHull[b],cHull[a],p2) > 0) return false;
            } else {
                //inside cone but not inside convex hull (3-point case)
                c = cHull.cyclic_succ(a);
                d = cHull.cyclic_succ(c);
                if (crossExtendedPoint(cHull[c],cHull[a],p2) > 0) return false;

                if (d == b) {
                    if (crossExtendedPoint(cHull[b],cHull[c],p2) > 0) return false;
                } else {
                    if (crossExtendedPoint(cHull[b],cHull[d],p2) > 0) return false;
                }
                if (crossExtendedPoint(cHull[d],cHull[c],p2) > 0) return false;
            }
            return true;
        }
        a = cHull.succ(a);
        b = cHull.cyclic_succ(b);
    } 

    return true;

}

void DaeWrenchFanChecker::genConvexHullExtended(const ExtendedPoint &p1,const ExtendedPoint &p2,
                           const ExtendedPoint &p3,const ExtendedPoint &p4,
                           ConvexHull &cHull)
{
    // think likes s1 = segment from p1 to p2
    //             s2 = segment from p3 to p4
    // main line cross value (first index: 0 -> (p1-p2), 1 -> (p3-p4), second index: 0 = the other segment start, 1 -> the other secgment end
    float crossValue[2][2];
    int i,j;
    int otherI,otherJ;

    crossValue[0][0] = crossExtendedPoint(p1,p2,p3);
    crossValue[0][1] = crossExtendedPoint(p1,p2,p4);
    crossValue[1][0] = crossExtendedPoint(p3,p4,p1);
    crossValue[1][1] = crossExtendedPoint(p3,p4,p2);

	/*
    //check colinear case
    leda_point *p1,*p2,*p3,*p4;

    for (i = 0,otherI = 1;i < 2;otherI = i , i++) {
        for (j = 0,otherJ = 1;j < 2;otherJ = j , j++) {
            if (crossValue[i][j] == 0) {
                if (i == 0) {
                    p1 = &(s1.start());
                    p2 = &(s1.end());
                    if (j == 0) {
                        p3 = &(s2.start()); 
                        p3 = &(s2.end()); 
                    } else {
                        p3 = &(s2.end());
                        p4 = &(s2.start()); 
                    }
                } else {
                    p1 = &(s2.start());
                    p2 = &(s2.end());
                    if (j == 0) {
                        p3 = &(s1.start()); 
                        p4 = &(s1.end()); 
                    } else {
                        p3 = &(s1.end());
                        p4 = &(s1.start()); 
                    }
                }

                // check all-colinear
                if (crossValue[i][otherJ] == 0) {
                    return ;    // all colinear
                } 

                genConvexHull_colinear_case(p1,p2,p3,p4,cHull);

                return ; //all done
            }
        }
    }
    // solve non-colinear
	*/

    //check case
    int crossingCase;
    if (crossValue[0][0] * crossValue[0][1] > 0) {         //same sign, s2 is entirely on one side of s1
        if (crossValue[1][0] * crossValue[1][1] > 0) {     //same sign, s1 is entirely on one side of s2
            crossingCase = 1; //CASE1 : both of them have same sign
        } else {
            crossingCase = 2; //CASE2 : one end point of s2 has to be removed
        }
    } else {    // diff sign s2 is on the different side of s1
        if (crossValue[1][0] * crossValue[1][1] > 0) {     //same sign, s1 is entirely on one side of s2
            crossingCase = 3; //CASE3 : one end point of s1 has to be removed
        } else {
            crossingCase = 4; //CASE3 : line cross over
        }
    }

    if (crossingCase == 1) {
        if (crossValue[0][0] > 0) {
            cHull.add(p1); cHull.add(p2);
        } else {
            cHull.add(p2); cHull.add(p1);
        }
        if (crossValue[1][0] > 0) {
            cHull.add(p3); cHull.add(p4);
        } else {
            cHull.add(p4); cHull.add(p3);
        }
    } else if (crossingCase == 2) {
        int secondIndexToCheck;
        if (crossValue[0][0] > 0) {
            cHull.add(p1); cHull.add(p2); secondIndexToCheck = 1;
        } else {
            cHull.add(p2); cHull.add(p1); secondIndexToCheck = 0;
        }
        if (crossValue[1][secondIndexToCheck] > 0) {
            cHull.add(p3);
        } else {
            cHull.add(p4);
        }
    } else if (crossingCase == 3) {
        int secondIndexToCheck;
        if (crossValue[1][0] > 0) {
            cHull.add(p3); cHull.add(p4); secondIndexToCheck = 1;
        } else {
            cHull.add(p4); cHull.add(p3); secondIndexToCheck = 0;
        }
        if (crossValue[0][secondIndexToCheck] > 0) {
            cHull.add(p1);
        } else {
            cHull.add(p2);
        }
    } else if (crossingCase == 4) {
        if (crossValue[0][0] > 0) {
            cHull.add(p1);
            cHull.add(p4);
            cHull.add(p2);
            cHull.add(p3);
        } else {
            cHull.add(p1);
            cHull.add(p3);
            cHull.add(p2);
            cHull.add(p4);
        }
    }
}

float DaeWrenchFanChecker::crossExtendedPoint(const ExtendedPoint& p1,const ExtendedPoint& p2,const ExtendedPoint& p3)
{
    //find cross product of p1p2 & p1p3
    float x1,y1,x2,y2;
    bool negate = false;
    if (p1.isPai == 0) {
        if (p2.isPai == 0) {
            x1 = p2.point.x - p1.point.x;
            y1 = p2.point.y - p1.point.y;
        } else {
            //p2 is a point at infinity, use the direction of p2, neglect p1
            x1 = p2.point.x;
            y1 = p2.point.y;
        }
        if (p3.isPai == 0) {
            x2 = p3.point.x - p1.point.x;
            y2 = p3.point.y - p1.point.y;
        } else {
            //p3 is a point at infinity, use the direction of p2, neglect p1
            x2 = p3.point.x;
            y2 = p3.point.y;
        }
    } else {
        //p1 is a ray, the result should be negated
        negate = true;
        
        if (p2.isPai == 0) {
            //since p1 is a point at infinity, a normal p2 is neglected
            x1 = p1.point.x;
            y1 = p1.point.y;
        } else {
            //if p2 is also a point-at-infinity, we return the direction of p1 cross p2 (in case that p3 is not a point at infinity
            if (p3.isPai == 0) {
                return p1.point.x * p2.point.y - p1.point.y * p2.point.x;
            } else {
                // every point is on the extended line.....
                //float cross1 = p1.point.x * p2.point.y - p1.point.y * p2.point.x;
                //float cross2 = p2.point.x * p3.point.y - p2.point.y * p3.point.x;
                //if (cross1 > 0 && cross2 > 0) return 1.f; else return -1.f;
                return 0;
            }
        }


        //atthis point, p2 is surely a point
        if (p3.isPai == 0) {
            //we use vector from p2 instead, since the result is negated...
            x2 = p3.point.x - p2.point.x;
            y2 = p3.point.y - p2.point.y;
        } else {
            //p3 is also a point at infinity, use the direction of p3 (which is p2 in this case)
            x2 = p3.point.x;
            y2 = p3.point.y;
        }
    }
    return (negate)? -(x1 * y2 - x2 * y1) : x1 * y2 - x2 * y1;
}


