#include "DaeHeuristic.h"
#include "daeUtils.h"
#include "Utils.h"
#include <math.h>
#include <float.h>

typedef SmallStore<mvPoint3d> ConvexHull;
typedef GP_DATATYPE* SingleWrench;


DaeHeuristicChecker::DaeHeuristicChecker(double halfAngleRad,int maxWrench)
{
    setHalfAngle(halfAngleRad);

    /*
    yhl = new YHLPosSpanTest(3,maxWrench);
    gjk = new GJKPosSpanTest(3,maxWrench);
    qdist = new QDistPosSpanTest(3,maxWrench);
    qhull = new QHPosSpanTest(3,maxWrench);
    */


    wrenches = new SingleWrench[maxWrench];
    vect = new vector3[maxWrench];
    for (int i = 0;i < maxWrench;++i) {
        wrenches[i] = new GP_DATATYPE[3];
    }

}

DaeHeuristicChecker::~DaeHeuristicChecker()
{
    /*
    delete yhl;
    delete gjk;
    delete qdist;
    delete qhull;
    */
    delete [] wrenches;
    delete [] vect;
}

void DaeHeuristicChecker::setHalfAngle(double halfAngleRad)
{
    this->halfAngle = halfAngleRad;
    this->fullAngle = halfAngleRad * 2;

    sinHalfAngle = sin(halfAngleRad);
    cosHalfAngle = cos(halfAngleRad);

    sinMinusHalfAngle = -sinHalfAngle;
    cosMinusHalfAngle = cosHalfAngle;

    sinFullAngle = sin(fullAngle);
    cosFullAngle = cos(fullAngle);

    sinMinusFullAngle = -sinFullAngle;
    cosMinusFullAngle = cosFullAngle;
}

bool DaeHeuristicChecker::isPosSpan4vect(const mvPoint3d &v1,const mvPoint3d &v2,const mvPoint3d &v3,const mvPoint3d &v4)
{
    mvPoint3d n1,n2,n3;

    n1 = v1 * v2;
    double v3Side = v3 ^ n1;
    if (v3Side > 0) {
        //order is v1,v2,v3
        if ((v4 ^ n1) >= 0) return false;

        n2 = v2 * v3;
        if ((v4 ^ n2) >= 0) return false;

        n3 = v3 * v1;
        if ((v4 ^ n3) >= 0) return false;

        return true;
    } else if (v3Side < 0) {
        //order is v2,v1,v3
        n1 = -n1;
        if ((v4 ^ n1) >= 0) return false;

        n2 = v1 * v3;
        if ((v4 ^ n2) >= 0) return false;

        n3 = v3 * v2;
        if ((v4 ^ n3) >= 0) return false;

        return true;
    } else {
        //collinear cae,
        return false;
    }

}

bool genConvexHull(const mvPoint3d &v1,const mvPoint3d &v2,const mvPoint3d &v3,const mvPoint3d &v4,
                   ConvexHull& cHull)
{
    float crossValue[2][2];
    int i,j;
    int otherI,otherJ;

    mvPoint3d norm12;           //normal vector of a plane containing v1&v2
    mvPoint3d norm34;           //normal vector of a plane containing v3&v4

    norm12 = v1 * v2;
    norm34 = v3 * v4;

    crossValue[0][0] = v3 ^ norm12;
    crossValue[0][1] = v4 ^ norm12;
    crossValue[1][0] = v1 ^ norm34;
    crossValue[1][1] = v2 ^ norm34;


    //check colinear case
    /*
    for (i = 0,otherI = 1;i < 2;otherI = i , i++) {
        for (j = 0,otherJ = 1;j < 2;otherJ = j , j++) {
            if (crossValue[i][j] == 0) {
                if (i == 0 && j == 0) {
                    findEndPointsOfCollinear(p1,p2,p3,cHull);
                    if (cHull.size() == 3) return true;
                    addPointToLineConvex(p4,cHull);
                } else if (i == 0 && j == 1) {
                    findEndPointsOfCollinear(p1,p2,p4,cHull);
                    if (cHull.size() == 3) return true;
                    addPointToLineConvex(p3,cHull);
                } else if (i == 1 && j == 0) {
                    findEndPointsOfCollinear(p3,p4,p1,cHull);
                    if (cHull.size() == 3) return true;
                    addPointToLineConvex(p2,cHull);
                } else if (i == 1 && j == 1) {
                    findEndPointsOfCollinear(p3,p4,p2,cHull);
                    if (cHull.size() == 3) return true;
                    addPointToLineConvex(p1,cHull);
                }
                return false; //all done
            }
        }
    }
    */


    // solve non-colinear
    //check case
    int crossingCase;
    if (crossValue[0][0] * crossValue[0][1] > 0) {         //same sign, v3,v4 is on the same side of norm12
        if (crossValue[1][0] * crossValue[1][1] > 0) {     //same sign, v1,v2 is on the same side of norm34
            crossingCase = 1; //CASE1 : both of them have same sign
        } else {                                           //diff sign, v1,v2 is on the different side of norm34
            crossingCase = 2; //CASE2 : one end point of s2 has to be removed
        }
    } else {    // diff sign v3,v4 is on the different side of norm12
        if (crossValue[1][0] * crossValue[1][1] > 0) {     //same sign, s1 is entirely on one side of s2
            crossingCase = 3; //CASE3 : one end point of s1 has to be removed
        } else {                                           //diff sign, v1,v2 is on the different side of norm34
            crossingCase = 4; //CASE3 : line cross over
        }
    }

    if (crossingCase == 1) {
        if (crossValue[0][0] > 0) {
            cHull.add(v1); cHull.add(v2);
        } else {
            cHull.add(v2); cHull.add(v1);
        }
        if (crossValue[1][0] > 0) {
            cHull.add(v3); cHull.add(v4);
        } else {
            cHull.add(v4); cHull.add(v3);
        }
    } else if (crossingCase == 2) {
        int secondIndexToCheck;
        if (crossValue[0][0] > 0) {
            cHull.add(v1); cHull.add(v2); secondIndexToCheck = 1;
        } else {
            cHull.add(v2); cHull.add(v1); secondIndexToCheck = 0;
        }
        if (crossValue[1][secondIndexToCheck] > 0) {
            cHull.add(v3);
        } else {
            cHull.add(v4);
        }
    } else if (crossingCase == 3) {
        int secondIndexToCheck;
        if (crossValue[1][0] > 0) {
            cHull.add(v3); cHull.add(v4); secondIndexToCheck = 1;
        } else {
            cHull.add(v4); cHull.add(v3); secondIndexToCheck = 0;
        }
        if (crossValue[0][secondIndexToCheck] > 0) {
            cHull.add(v1);
        } else {
            cHull.add(v2);
        }
    } else if (crossingCase == 4) {
        if (crossValue[0][0] > 0) {
            cHull.add(v1);
            cHull.add(v4);
            cHull.add(v2);
            cHull.add(v3);
        } else {
            cHull.add(v1);
            cHull.add(v3);
            cHull.add(v2);
            cHull.add(v4);
        }
    }

    return true;
}

bool DaeHeuristicChecker::isPosSpan6vect(const mvPoint3d &v1,const mvPoint3d &v2,
					                     const mvPoint3d &v3,const mvPoint3d &v4,
                                         const mvPoint3d &v5,const mvPoint3d &v6)
{
    if (isPosSpan4vect(v1,v2,v3,v4)) {
        return true;
    } else {
        //generate convex hull of v1..v4
        int i,j;
        ConvexHull cHull;
        genConvexHull(v1,v2,v3,v4,cHull);

        /*
        //check whether v5 is inside c1,c2,c3
        if (isPosSpan4vect(cHull[0],cHull[1],cHull[2],v5)) return true;

        //check whether v5 is inside c3,c4,c1 (if needed)
        if (cHull.getSize() >= 4 &&
            isPosSpan4vect(cHull[2],cHull[3],cHull[0],v5)) return true;
            */
        bool inside = true;
        for (i = 0;i < cHull.getSize();++i) {
            j = (i+1 == cHull.getSize()) ? 0 : i+1;
            if (((cHull[i]*cHull[j]) ^ v5) > 0) {
                inside = false;
                break;
            }
        }
        if (inside) return true;



        //check whether fan v5,v6 intersects with any boundary
        for (i = 0;i < cHull.getSize();++i) {
            j = (i+1 == cHull.getSize()) ? 0 : i+1;
            if (isPosSpan4vect(cHull[i],cHull[j],v5,v6)) return true;

        }
        return false;
    }
}


bool DaeHeuristicChecker::isForceConeSpan2(const mvPoint3d &v1,const mvPoint3d &v2)
{
    double angle = acos(v1 ^ v2);
    return (angle > M_PI - fullAngle);
}

bool DaeHeuristicChecker::isInMiddleZone(const mvPoint3d &v1,const mvPoint3d &v2,const mvPoint3d &v3)
{
    //check the middle area defined by  v1 and v2 (v1 is left side)
    mvPoint3d leftNorm,rightNorm,topNorm,bottomNorm;
    mvPoint3d leftTop,leftBottom,rightTop,rightBottom;
    mvPoint3d v = -v3;
    mvMatrix44d rotMat;

    leftNorm = GramSchmidt(v1,v2,false);
    if ((v ^ leftNorm) <= 0) return false;

    rightNorm = GramSchmidt(v2,v1,false);
    if ((v ^ rightNorm) <= 0) return false;

    leftNorm = leftNorm.normalize();
    rightNorm = rightNorm.normalize();
    genRot(leftNorm,rotMat,cosMinusFullAngle,sinMinusFullAngle);
    leftTop = rotMat * v1;
    genRot(rightNorm,rotMat,cosFullAngle,sinFullAngle);
    rightTop = rotMat * v2;
    topNorm = rightTop * leftTop;
    if ((v ^ topNorm) <= 0) return false;

    genRot(leftNorm,rotMat,cosFullAngle,sinFullAngle);
    leftBottom = rotMat * v1;
    genRot(rightNorm,rotMat,cosMinusFullAngle,sinMinusFullAngle);
    rightBottom = rotMat * v2;
    bottomNorm = leftBottom * rightBottom;
    if ((v ^ bottomNorm) <= 0) return false;

    /* DEBUG
    printf("\n");
    printf("%f %f %f\n",topNorm.x,topNorm.y,topNorm.z);
    printf("%f %f %f\n",bottomNorm.x,bottomNorm.y,bottomNorm.z);
    printf("%f %f %f\n",leftNorm.x,leftNorm.y,leftNorm.z);
    printf("%f %f %f\n",rightNorm.x,rightNorm.y,rightNorm.z);
    if ((v ^ leftNorm) <= 0) return false;
    if ((v ^ bottomNorm) <= 0) return false;
    if ((v ^ topNorm) <= 0) return false;
    if ((v ^ rightNorm) <= 0) return false;


    printf("glVertex3f(%f,%f,%f);\n",leftTop.x,leftTop.y,leftTop.z);
    printf("glVertex3f(%f,%f,%f);\n",rightTop.x,rightTop.y,rightTop.z);
    printf("glVertex3f(%f,%f,%f);\n",rightBottom.x,rightBottom.y,rightBottom.z);
    printf("glVertex3f(%f,%f,%f);\n",leftBottom.x,leftBottom.y,leftBottom.z);

    float top =  v ^ topNorm;
    float left =  v ^ leftNorm;
    float right =  v ^ rightNorm;
    float bottom =  v ^ bottomNorm;
    printf("top woi %f\n",top);
    printf("left woi %f\n",left);
    printf("right woi %f\n",right);
    printf("bottom woi %f\n",bottom);
    if ((v ^ leftNorm) <= -0.0) return false;
    if ((v ^ rightNorm) <= -0.0) return false;
    if ((v ^ topNorm) <= -0.0) return false;
    if ((v ^ bottomNorm) <= -0.0) return false;
    */



    return true;
}

bool DaeHeuristicChecker::isForceConeSpan3(const mvPoint3d &v1,const mvPoint3d &v2,const mvPoint3d &v3)
{
    //check whether v1 lies inside v2

    if (isForceConeSpan2(v1,v2)) {
        return true;
    } else {
        if (isForceConeSpan2(v3,v1)) return true;
        if (isForceConeSpan2(v3,v2)) return true;

        return isInMiddleZone(v1,v2,v3);
        /*
        if (isInMiddleZone(v1,v2,v3)) return true;
        if (isInMiddleZone(v1,v3,v2)) return true;
        if (isInMiddleZone(v2,v3,v1)) return true;
        return false;
        */
    }
}

bool DaeHeuristicChecker::isForceConeSpan4(const mvPoint3d &v1,const mvPoint3d &v2,const mvPoint3d &v3,const mvPoint3d &v4)
{
    if (isForceConeSpan3(v1,v2,v3)) {
        return true;
    } else {
        if (isForceConeSpan2(v1,v4)) return true;
        if (isForceConeSpan2(v2,v4)) return true;
        if (isForceConeSpan2(v3,v4)) return true;
        if (isInMiddleZone(v1,v2,v4)) return true;
        if (isInMiddleZone(v2,v3,v4)) return true;
        if (isInMiddleZone(v3,v1,v4)) return true;

        /*
        if (isForceConeSpan3(v1,v2,v4))  return true;
        if (isForceConeSpan3(v1,v3,v4))  return true;
        if (isForceConeSpan3(v2,v3,v4))  return true;
        */

        return isPosSpan4vect(v1,v2,v3,v4);
        //return true;

    }
}

void DaeHeuristicChecker::generateTorqueFan(
    const mvPoint3d &pos,const mvPoint3d &norm,
    mvPoint3d &left,mvPoint3d &right
) {

}


/**
 * Check whether [v1],[v2],[v3],[v4] positively span, given a plane whose normal is [normal]
 * this is done by checking whether [v1..v4] are on the different side of the plane
 */
bool onDiffSide(const mvPoint3d &normal,
                const mvPoint3d &v1,const mvPoint3d &v2,
                const mvPoint3d &v3,const mvPoint3d &v4
) {
    bool hasPos = false;
    bool hasNeg = false;

    float s1 = normal ^ v1;
    float s2 = normal ^ v2;
    float s3 = normal ^ v3;
    float s4 = normal ^ v4;

    if (s1 > 0) hasPos = true; if (s1 < 0) hasNeg = true;
    if (s2 > 0) hasPos = true; if (s2 < 0) hasNeg = true;
    if (s3 > 0) hasPos = true; if (s3 < 0) hasNeg = true;
    if (s4 > 0) hasPos = true; if (s4 < 0) hasNeg = true;

    return hasPos && hasNeg;
}


#include "Display.h"

bool DaeHeuristicChecker::isTorqueSpan(
    const mvPoint3d &origin,
    GraspingPoint3D &gp1,
    GraspingPoint3D &gp2,
    GraspingPoint3D &gp3
) {
    gp1.genTorqueFan(origin.x,origin.y,origin.z);
    gp2.genTorqueFan(origin.x,origin.y,origin.z);
    gp3.genTorqueFan(origin.x,origin.y,origin.z);

    TorqueFan tf1 = gp1.torqueFan;
    TorqueFan tf2 = gp2.torqueFan;
    TorqueFan tf3 = gp3.torqueFan;

    //SchemeOutput scm("tf.scm");
    //if (tf1.isPlane) {
    //    scm.print_plane(mvPoint3d(0,0,0),tf1.normal);
    //} else {
    //    scm.print_line(mvPoint3d(0,0,0),tf1.v1 * 100);
    //    scm.print_line(mvPoint3d(0,0,0),tf1.v2 * 100);
    //}
    //if (tf2.isPlane) {
    //    scm.print_plane(mvPoint3d(0,0,0),tf2.normal);
    //} else {
    //    scm.print_line(mvPoint3d(0,0,0),tf2.v1 * 100);
    //    scm.print_line(mvPoint3d(0,0,0),tf2.v2 * 100);
    //}
    //if (tf3.isPlane) {
    //    scm.print_plane(mvPoint3d(0,0,0),tf3.normal);
    //} else {
    //    scm.print_line(mvPoint3d(0,0,0),tf3.v1 * 100);
    //    scm.print_line(mvPoint3d(0,0,0),tf3.v2 * 100);
    //}
    //scm.close();


    //check for special case where torqueFan is actually a plane

    //check if plane exists
    mvPoint3d planeVector;
    bool hasPlane = false;
    if (gp1.torqueFan.isPlane) {
        hasPlane = true;
        planeVector = gp1.torqueFan.normal;

        if ( gp1.torqueFan.crossPlane(gp2.torqueFan) || gp1.torqueFan.crossPlane(gp3.torqueFan) ) return true;
        if ( gp2.torqueFan.isPlane == false        && gp3.torqueFan.isPlane == false ) {
            float d1 = gp2.torqueFan.v1 ^ planeVector;
            float d2 = gp3.torqueFan.v1 ^ planeVector;

            return (d1 * d2 < 0);
        }
        return false;
    } else if (gp2.torqueFan.isPlane) {
        hasPlane = true;
        planeVector = gp2.torqueFan.normal;

        if ( gp2.torqueFan.crossPlane(gp1.torqueFan) || gp2.torqueFan.crossPlane(gp3.torqueFan) ) return true;
        if ( gp1.torqueFan.isPlane == false        && gp3.torqueFan.isPlane == false ) {
            float d1 = gp1.torqueFan.v1 ^ planeVector;
            float d2 = gp3.torqueFan.v1 ^ planeVector;

            return (d1 * d2 < 0);
        }
        return false;
    } else if (gp3.torqueFan.isPlane) {
        hasPlane = true;
        planeVector = gp3.torqueFan.normal;

        if ( gp3.torqueFan.crossPlane(gp1.torqueFan) || gp3.torqueFan.crossPlane(gp2.torqueFan) ) return true;
        if ( gp2.torqueFan.isPlane == false        && gp1.torqueFan.isPlane == false ) {
            float d1 = gp2.torqueFan.v1 ^ planeVector;
            float d2 = gp1.torqueFan.v1 ^ planeVector;
            float d3 = gp2.torqueFan.v2 ^ planeVector;
            float d4 = gp1.torqueFan.v2 ^ planeVector;

            return (d1 * d2 < 0);
        }
        return false;
    }


    //check tf1
    /*
    if (tf1.isPlane) {
        if (tf2.isPlane) {
            if (tf1.crossPlane(tf2.normal)) return true;

            //both tf1 and tf2 are planes and are coplanar
            return tf1.crossPlane(tf3);
        } else {
            //tf1 is plane while tf2 is not
            if (tf1.crossPlane(tf2)) return true;

            //tf2 is on the same side of tf1
            if (tf3.isPlane) {
                //tf3 is plane, hence, it is possitively span only when tf3 crosses tf1, regardless of tf2
                return (tf1.crossPlane(tf3.normal));
            } else {
                //tf3 is not a plane
                if (tf1.crossPlane(tf3)) return true;
            }

            //tf2 and tf3 both is not a plane, each is on the same side of tf1
            //it is pos span iff tf2 and tf3 is on the different side of tf1
            return onDiffSide(tf1.normal,
                              tf2.v1,tf2.v2,
                              tf3.v1,tf3.v2);
        }
    }

    //check tf2, now tf1 is definitely a fan
    if (tf2.isPlane) {
        if (tf2.crossPlane(tf1)) return true;

        //tf1  is on the same side of tf2
        if (tf3.isPlane) {
            //tf3 is plane, hence, it is possitively span only when tf3 crosses tf2, regardless of tf1
            return tf2.crossPlane(tf1);
        } else {
            //tf3 is not a plane
            if (tf2.crossPlane(tf3.normal)) return true;
        }

        //tf1 and tf3 both is not a plane, each is on the same side of tf2
        //it is pos span iff tf1 and tf3 is on the different side of tf2
        return onDiffSide(tf2.normal,
                          tf1.v1,tf1.v2,
                          tf3.v1,tf3.v2);
    }

    //check tf3, now tf1 and tf2 are definite fans
    if (tf3.isPlane) {
        if (tf3.crossPlane(tf1)) return true;
        if (tf3.crossPlane(tf2)) return true;

        return onDiffSide(tf3.normal,
                          tf1.v1,tf2.v2,
                          tf2.v1,tf2.v2);
    }
    */

    return isPosSpan6vect(tf1.v1,tf1.v2,
                          tf2.v1,tf2.v2,
                          tf3.v1,tf3.v2);
}
/*
bool DaeHeuristicChecker::isTorqueSpan(const mvPoint3d &origin,int numGP,GraspingPoint3D** gpList)
{
    for (int i = 0;i < numGP;i++) {
        gpList[i]->genTorqueFan(origin.x,origin.y,origin.z);
    }

    int count = 0;
    for (int i = 0;i < numGP;i++) {
        if (gpList[i]->torqueFan.isPlane) {
            bool hasPositive = false;
            bool hasNegative = false;
            mvPoint3d planeVector = gpList[i]->torqueFan.normal;
            for (int j = 0;j < numGP; j++) {
                if (i == j) continue;
                if (gpList[j]->torqueFan.isPlane) {
                    // j is not a plane
                    double d1 = gpList[j]->torqueFan.v1 ^ planeVector;
                    if (d1 > 0) hasPositive = true;
                    if (d1 < 0) hasNegative = true;

                    double d2 = gpList[j]->torqueFan.v2 ^ planeVector;
                    if (d2 > 0) hasPositive = true;
                    if (d2 < 0) hasNegative = true;

                    if (hasPositive && hasNegative) {
                        return true;
                    }
                } else {
                    // j is also a plane
                    if (j > i) {
                        if (gpList[j]->torqueFan.crossPlane(gpList[i]->torqueFan.normal)) {
                            return true;
                        }
                    }
                }
            }
        } else {
            // gpList[i] is not a plane
            vect[count++] = gpList[i]->torqueFan.v1;
            vect[count++] = gpList[i]->torqueFan.v2;
        }
    }

    //return true;
    return isPosSpanNVect(count,vect);

}
*/
/*
bool DaeHeuristicChecker::isPosSpanNVect(int numVect,vector3 *vect)
{
    for (int i = 0;i < numVect; ++i) {
        wrenches[i][0] = vect[i].x;
        wrenches[i][1] = vect[i].y;
        wrenches[i][2] = vect[i].z;
    }


    //return qhull->isForceClosure(numVect,3,wrenches);
    //return qdist->isForceClosure(numVect,3,wrenches);
    //return gjk->isForceClosure(numVect,3,wrenches);
    return yhl->isForceClosure(numVect,3,wrenches);
}
*/


bool DaeHeuristicChecker::isForceClosure(
    GraspingPoint3D &gp1,
    GraspingPoint3D &gp2,
    GraspingPoint3D &gp3,
    GraspingPoint3D &gp4
) {
    //check force part
    if (isForceConeSpan4(gp1.getNormal(),gp2.getNormal(),gp3.getNormal(),gp4.getNormal()) == false) return false;


    //check torque
    if (isTorqueSpan(gp1.getPosition(),gp2,gp3,gp4) == false) return false;
    if (isTorqueSpan(gp2.getPosition(),gp1,gp3,gp4) == false) return false;
    if (isTorqueSpan(gp3.getPosition(),gp1,gp2,gp4) == false) return false;
    if (isTorqueSpan(gp4.getPosition(),gp1,gp2,gp3) == false) return false;

    /*
    isTorqueSpan(gp1.getPosition(),gp2,gp3,gp4);
    mvPoint3d origin = gp1.getPosition();
    FILE *fp = fopen("gl.txt","w");

    fprintf(fp,"glColor3f(0,1,0);\n");
    gp3.genWrenches(origin.x,origin.y,origin.z);
    gp3.printTorqueConeGL(fp);
    fprintf(fp,"glColor3f(0,0.5,0.5);\nglBegin(GL_TRIANGLES);\nglVertex3f(0,0,0);\n");
    fprintf(fp,"glVertex3f(%f,%f,%f);\n",gp3.torqueFan.v1.x,gp3.torqueFan.v1.y,gp3.torqueFan.v1.z);
    fprintf(fp,"glVertex3f(%f,%f,%f);\n",gp3.torqueFan.v2.x,gp3.torqueFan.v2.y,gp3.torqueFan.v2.z);
    fprintf(fp,"glEnd();\n");

    fprintf(fp,"glColor3f(1,0,0);\n");
    gp2.genWrenches(origin.x,origin.y,origin.z);
    gp2.printTorqueConeGL(fp);
    fprintf(fp,"glColor3f(1,0.5,0);\nglBegin(GL_TRIANGLES);\nglVertex3f(0,0,0);\n");
    fprintf(fp,"glVertex3f(%f,%f,%f);\n",gp2.torqueFan.v1.x,gp2.torqueFan.v1.y,gp2.torqueFan.v1.z);
    fprintf(fp,"glVertex3f(%f,%f,%f);\n",gp2.torqueFan.v2.x,gp2.torqueFan.v2.y,gp2.torqueFan.v2.z);
    fprintf(fp,"glEnd();\n");

    fprintf(fp,"glColor3f(0,0,1);\n");
    gp4.genWrenches(origin.x,origin.y,origin.z);
    gp4.printTorqueConeGL(fp);
    fprintf(fp,"glColor3f(0.5,0,1);\nglBegin(GL_TRIANGLES);\nglVertex3f(0,0,0);\n");
    fprintf(fp,"glVertex3f(%f,%f,%f);\n",gp4.torqueFan.v1.x,gp4.torqueFan.v1.y,gp4.torqueFan.v1.z);
    fprintf(fp,"glVertex3f(%f,%f,%f);\n",gp4.torqueFan.v2.x,gp4.torqueFan.v2.y,gp4.torqueFan.v2.z);
    fprintf(fp,"glEnd();\n");
    fclose(fp);
    */

    return true;
}
/*
bool DaeHeuristicChecker::isForceClosure(int numGP,GraspingPoint3D** gpList)
{
    GraspingPoint3D** oneOffList = new GraspingPoint3D*[numGP];

    //select the \vq
    int minIdx = 0;
    for (int i = 0;i < numGP;i++) {
        if (gpList[i]->getPosition().x < gpList[minIdx]->getPosition().x) {
            minIdx = i;
        }
    }

    vector3 vq = gpList[minIdx]->getPosition() - vector3(1,0,0);

    //select the furtest and closest point;
    double minDist = DBL_MAX;
    double maxDist = 0;
    int minDistIdx,maxDistIdx;

    for (int i = 0;i < numGP;i++) {
        double dist = (vq - gpList[i]->getPosition()).getSqrSize();
        if ( dist > maxDist ) {
            maxDist = dist;
            maxDistIdx = i;
        }
        if (dist < minDist) {
            minDist = dist;
            minDistIdx = i;
        }
    }


    //check with min
    int count;
    count = 0;
    for (int j = 0;j < numGP;j++) {
        if (minDistIdx != j) {
            oneOffList[count++] = gpList[j];
        }
    }
    if (isTorqueSpan(gpList[minDistIdx]->getPosition(),numGP-1,oneOffList) == false) {
            return false;
    }

    count = 0;
    for (int j = 0;j < numGP;j++) {
        if (maxDistIdx != j) {
            oneOffList[count++] = gpList[j];
        }
    }
    if (isTorqueSpan(gpList[maxDistIdx]->getPosition(),numGP-1,oneOffList) == false) {
            return false;
    }

    return true;
}
*/
/*
bool DaeHeuristicChecker::isForceSpan4ByYHL(
    GraspingPoint3D &gp1,
    GraspingPoint3D &gp2,
    GraspingPoint3D &gp3,
    GraspingPoint3D &gp4
) {

    int wrenchCount = 0;
    int i;

    GraspingPoint3D* currGp;

    for (int count = 0;count < 4;++count) {
        switch (count) {
        case 0: currGp = &gp1; break;
        case 1: currGp = &gp2; break;
        case 2: currGp = &gp3; break;
        case 3: currGp = &gp4; break;
        }

        for (int i = 0;i < currGp->getWrenchCount(); ++i) {
            memcpy(wrenches[wrenchCount],*currGp->getWrench(i),sizeof(GP_DATATYPE) * 3);
            wrenchCount++;
        }
    }

    return yhl->isForceClosure(wrenchCount,3,(GP_DATATYPE**)wrenches);
}


bool DaeHeuristicChecker::isForceSpan3ByYHL(
    GraspingPoint3D &gp1,
    GraspingPoint3D &gp2,
    GraspingPoint3D &gp3
) {

    int wrenchCount = 0;
    int i;

    GraspingPoint3D* currGp;

    for (int count = 0;count < 3;++count) {
        switch (count) {
        case 0: currGp = &gp1; break;
        case 1: currGp = &gp2; break;
        case 2: currGp = &gp3; break;
        }

        for (int i = 0;i < currGp->getWrenchCount(); ++i) {
            memcpy(wrenches[wrenchCount],*currGp->getWrench(i),sizeof(GP_DATATYPE) * 3);
            wrenchCount++;
        }
    }

    return yhl->isForceClosure(wrenchCount,3,(GP_DATATYPE**)wrenches);
}

bool DaeHeuristicChecker::isForceSpan2ByYHL(
    GraspingPoint3D &gp1,
    GraspingPoint3D &gp2
) {

    int wrenchCount = 0;
    int i;

    GraspingPoint3D* currGp;

    for (int count = 0;count < 2;++count) {
        switch (count) {
        case 0: currGp = &gp1; break;
        case 1: currGp = &gp2; break;
        }

        for (int i = 0;i < currGp->getWrenchCount(); ++i) {
            memcpy(wrenches[wrenchCount],currGp->getWrench(i),sizeof(GP_DATATYPE) * 3);
            wrenchCount++;
        }
    }

    return yhl->isForceClosure(wrenchCount,3,(GP_DATATYPE**)wrenches);
}
*/
/*
bool DaeHeuristicChecker::isForceSpan4ByGJK(
    GraspingPoint3D &gp1,
    GraspingPoint3D &gp2,
    GraspingPoint3D &gp3,
    GraspingPoint3D &gp4
) {

    int wrenchCount = 0;
    int i;

    GraspingPoint3D* currGp;

    for (int count = 0;count < 4;++count) {
        switch (count) {
        case 0: currGp = &gp1; break;
        case 1: currGp = &gp2; break;
        case 2: currGp = &gp3; break;
        case 3: currGp = &gp4; break;
        }

        for (int i = 0;i < currGp->getWrenchCount(); ++i) {
            memcpy(wrenches[wrenchCount],*currGp->getWrench(i),sizeof(GP_DATATYPE) * 3);
            //printf("%f %f %f\n",wrenches[wrenchCount][0],wrenches[wrenchCount][1],wrenches[wrenchCount][2]);
            wrenchCount++;
        }
    }

    return gjk->isForceClosure(wrenchCount,3,(GP_DATATYPE**)wrenches);
}


bool DaeHeuristicChecker::isForceSpan3ByGJK(
    GraspingPoint3D &gp1,
    GraspingPoint3D &gp2,
    GraspingPoint3D &gp3
) {

    int wrenchCount = 0;
    int i;

    GraspingPoint3D* currGp;

    for (int count = 0;count < 3;++count) {
        switch (count) {
        case 0: currGp = &gp1; break;
        case 1: currGp = &gp2; break;
        case 2: currGp = &gp3; break;
        }

        for (int i = 0;i < currGp->getWrenchCount(); ++i) {
            memcpy(wrenches[wrenchCount],*currGp->getWrench(i),sizeof(GP_DATATYPE) * 3);
            wrenchCount++;
        }
    }

    return gjk->isForceClosure(wrenchCount,3,(GP_DATATYPE**)wrenches);
}

bool DaeHeuristicChecker::isForceSpan2ByGJK(
    GraspingPoint3D &gp1,
    GraspingPoint3D &gp2
) {

    int wrenchCount = 0;
    int i;

    GraspingPoint3D* currGp;

    for (int count = 0;count < 2;++count) {
        switch (count) {
        case 0: currGp = &gp1; break;
        case 1: currGp = &gp2; break;
        }

        for (int i = 0;i < currGp->getWrenchCount(); ++i) {
            memcpy(wrenches[wrenchCount],currGp->getWrench(i),sizeof(GP_DATATYPE) * 3);
            wrenchCount++;
        }
    }

    return gjk->isForceClosure(wrenchCount,3,(GP_DATATYPE**)wrenches);
}
*/
/*
bool DaeHeuristicChecker::isForceConeSpan2Test(
    GraspingPoint3D &gp1,
    GraspingPoint3D &gp2
) {
    double angle = acos(gp1.getNormal() ^ gp2.getNormal());
    return (angle > M_PI - fullAngle);
}

bool DaeHeuristicChecker::isForceConeSpan3Test(
    GraspingPoint3D &gp1,
    GraspingPoint3D &gp2,
    GraspingPoint3D &gp3
) {
    //check whether v1 lies inside v2
    if (isForceConeSpan2Test(gp1,gp2)) {
        return true;
    } else {
        if (isForceConeSpan2Test(gp3,gp1)) return true;
        if (isForceConeSpan2Test(gp3,gp2)) return true;

        printf("take here!!\n");
        isInMiddleZone(gp1.getNormal(),gp2.getNormal(),gp3.getNormal());
        printf("stop here!!\n");

        return isInMiddleZone(gp1.getNormal(),gp2.getNormal(),gp3.getNormal());
    }
}



bool DaeHeuristicChecker::isForceConeSpan4Test(
    GraspingPoint3D &gp1,
    GraspingPoint3D &gp2,
    GraspingPoint3D &gp3,
    GraspingPoint3D &gp4
) {
    if (isForceConeSpan3Test(gp1,gp2,gp3)) {
        return true;
    } else {
        if (isForceConeSpan2(gp1.getNormal(),gp4.getNormal())) return true;
        if (isForceConeSpan2(gp2.getNormal(),gp4.getNormal())) return true;
        if (isForceConeSpan2(gp3.getNormal(),gp4.getNormal())) return true;
        if (isInMiddleZone(gp1.getNormal(),gp2.getNormal(),gp4.getNormal())) return true;
        if (isInMiddleZone(gp2.getNormal(),gp3.getNormal(),gp4.getNormal())) return true;
        if (isInMiddleZone(gp3.getNormal(),gp1.getNormal(),gp4.getNormal())) return true;

        return isPosSpan4vect(gp1.getNormal(),gp2.getNormal(),gp3.getNormal(),gp4.getNormal());
        //return true;

    }
}
*/
bool DaeHeuristicChecker::checkTorque(
    vector3 newOrigin,
    GraspingPoint3D &gp1,
    GraspingPoint3D &gp2,
    GraspingPoint3D &gp3,
    GraspingPoint3D &gp4,
    GraspingPoint3D &gp5,
    GraspingPoint3D &gp6
) {
    mvPoint3d* points = new mvPoint3d[6];
    points[0] = (gp1.getPosition() - newOrigin) * gp1.getNormal();
    points[1] = (gp2.getPosition() - newOrigin) * gp2.getNormal();
    points[2] = (gp3.getPosition() - newOrigin) * gp3.getNormal();
    points[3] = (gp4.getPosition() - newOrigin) * gp4.getNormal();
    points[4] = (gp5.getPosition() - newOrigin) * gp5.getNormal();
    points[5] = (gp6.getPosition() - newOrigin) * gp6.getNormal();

    if (!is3DPosSpanBrute(points,6)) {
    //if (!isPosSpan6vect(points[0],points[1],points[2],points[3],points[4],points[5])) {
        delete [] points;
        return false;
    }

    return true;
}

/*
bool DaeHeuristicChecker::isForceClosure(
    GraspingPoint3D &gp1,
    GraspingPoint3D &gp2,
    GraspingPoint3D &gp3,
    GraspingPoint3D &gp4,
    GraspingPoint3D &gp5,
    GraspingPoint3D &gp6,
    GraspingPoint3D &gp7
) {
    // !!!!! in 3D case, we assumed that wrenches of each grasping point is already calculated !!!!!

    // check force
    mvPoint3d* points = new mvPoint3d[7];

    points[0] = gp1.getNormal();
    points[1] = gp2.getNormal();
    points[2] = gp3.getNormal();
    points[3] = gp4.getNormal();
    points[4] = gp5.getNormal();
    points[5] = gp6.getNormal();
    points[6] = gp7.getNormal();

    if (!is3DPosSpanBrute(points,7)) {
        delete [] points;
        return false;
    }

    if (!checkTorque(gp1.getPosition(),gp2,gp3,gp4,gp5,gp6,gp7)) { return false; }
    if (!checkTorque(gp2.getPosition(),gp1,gp3,gp4,gp5,gp6,gp7)) { return false; }
    if (!checkTorque(gp3.getPosition(),gp1,gp2,gp4,gp5,gp6,gp7)) { return false; }
    if (!checkTorque(gp4.getPosition(),gp1,gp2,gp3,gp5,gp6,gp7)) { return false; }
    if (!checkTorque(gp5.getPosition(),gp1,gp2,gp3,gp4,gp6,gp7)) { return false; }
    if (!checkTorque(gp6.getPosition(),gp1,gp2,gp3,gp4,gp5,gp7)) { return false; }
    if (!checkTorque(gp7.getPosition(),gp1,gp2,gp3,gp4,gp5,gp6)) { return false; }

    return true;
}
*/

int crossPoint(ReducedPoint& p1,ReducedPoint& p2,ReducedPoint& p3)
{

    //find cross product of p1p2 & p1p3
    double x1,y1,x2,y2,val;
    bool negate = false;
    if (p1.colorTag != 0) {
        if (p2.colorTag != 0) {
            x1 = p2.p.x - p1.p.x;
            y1 = p2.p.y - p1.p.y;
        } else {
            //p2 is a point at infinity, use the direction of p2, neglect p1
            x1 = p2.p.x;
            y1 = p2.p.y;
        }
        if (p3.colorTag != 0) {
            x2 = p3.p.x - p1.p.x;
            y2 = p3.p.y - p1.p.y;
        } else {
            //p3 is a point at infinity, use the direction of p2, neglect p1
            x2 = p3.p.x;
            y2 = p3.p.y;
        }
    } else {
        //p1 is a ray, the result should be negated
        negate = true;

        if (p2.colorTag != 0) {
            //since p1 is a point at infinity, a normal p2 is neglected
            x1 = p1.p.x;
            y1 = p1.p.y;
        } else {
            //if p2 is also a point-at-infinity, we return the direction of p1 cross p2 (in case that p3 is not a point at infinity
            if (p3.colorTag != 0) {
                val = p1.p.x * p2.p.y - p1.p.y * p2.p.x;
                if (val > 0) return 1; else if (val < 0) return -1; else return 0;
            } else {
                // every point is on the extended line.....
                //float cross1 = p1.p.x * p2.p.y - p1.p.y * p2.p.x;
                //float cross2 = p2.p.x * p3.p.y - p2.p.y * p3.p.x;
                //if (cross1 > 0 && cross2 > 0) return 1.f; else return -1.f;
                return 0;
            }
        }


        //atthis point, p2 is surely a point
        if (p3.colorTag != 0) {
            //we use vector from p2 instead, since the result is negated...
            x2 = p3.p.x - p2.p.x;
            y2 = p3.p.y - p2.p.y;
        } else {
            //p3 is also a point at infinity, use the direction of p3 (which is p2 in this case)
            x2 = p3.p.x;
            y2 = p3.p.y;
        }
    }
    val = (negate)? -(x1 * y2 - x2 * y1) : x1 * y2 - x2 * y1;
    //return val;
    if (val > 0) return 1; else if (val < 0) return -1; else return 0;
}


bool isIntersect(ReducedPoint& p1,ReducedPoint& p2,ReducedPoint& q1,ReducedPoint& q2)
{

    return (crossPoint(p1,p2,q1) * crossPoint(p1,p2,q2) <= 0) &&
           (crossPoint(q1,q2,p1) * crossPoint(q1,q2,p2) <= 0);
}

int partition(ReducedPointPtr *ch,int left,int right,int pivotIndex)
{
    ReducedPointPtr tmp;
    tmp = ch[pivotIndex];       // Move pivot to end
    ch[pivotIndex] = ch[right];
    ch[right] = tmp;
    int storeIndex = left;
    double crossValue;
    int i;
    bool more;
    for (i = left;i < right;++i) {
        //compare

        crossValue = crossPoint(*ch[0],*ch[i],*ch[right]);  //compare ch[i] with the pivot stored at ch[right]
        if (crossValue < 0) {
            more = true;            //angle of ch[i] is more than that of ch[right]
        } else if (crossValue == 0) {
            if ((ch[i]->p.x - ch[0]->p.x) * (ch[i]->p.x - ch[0]->p.x) +
                (ch[i]->p.y - ch[0]->p.y) * (ch[i]->p.y - ch[0]->p.y) >
                (ch[right]->p.x - ch[0]->p.x) * (ch[right]->p.x - ch[0]->p.x) +
                (ch[right]->p.y - ch[0]->p.y) * (ch[right]->p.y - ch[0]->p.y))
                more = true;
            else
                more = false;

        } else
            more = false;
        if (!more) {
            tmp = ch[storeIndex];
            ch[storeIndex] = ch[i];
            ch[i] = tmp;
            storeIndex++;
        }
    }

    tmp = ch[right];
    ch[right] = ch[storeIndex];
    ch[storeIndex] = tmp;

    return storeIndex;
}


//sort from ch[1] --> ch[numPoint-1] (ch[0] always is the minimum)
void doSort(int left,int right,ReducedPointPtr *ch)
{
    if (right > left) {
        int pIdx = (left + right) >> 1;
        int pIdxNew = partition(ch, left, right, pIdx);
        doSort(left, pIdxNew-1,ch);
        doSort(pIdxNew+1, right,ch);
    }
}


//find edge with smallest positive angle (return the index)
int findMinAngle(ReducedPointPtr* ch,int num,bool isPositive)
{
    double minAngle;
    double angle;
    minAngle = M_PI * 2;
    int i,j,minIdx;
    for (i = 0;i < num;++i) {
        j = (i + 1 == num) ? 0 : i + 1;
        if (ch[i]->colorTag == 0 && ch[j]->colorTag == 0) {
            if (isPositive) {
                angle = atan2(ch[j]->p.y - ch[i]->p.y,ch[j]->p.x - ch[i]->p.x);
            } else {
                angle = atan2(ch[i]->p.y - ch[j]->p.y,ch[i]->p.x - ch[j]->p.x);
            }
        } else if (ch[i]->colorTag == 0 && ch[j]->colorTag != 0) {
            if (isPositive) {
                angle = atan2( - ch[i]->p.y, - ch[i]->p.x);
            } else {
                angle = atan2(   ch[i]->p.y,   ch[i]->p.x);
            }
        } else if (ch[i]->colorTag != 0 && ch[j]->colorTag == 0) {
            if (isPositive) {
                angle = atan2(ch[j]->p.y,ch[j]->p.x);
            } else {
                angle = atan2( - ch[j]->p.y, - ch[j]->p.x);
            }
        } else {
            if (isPositive) {
                angle = atan2(ch[j]->p.y - ch[i]->p.y,ch[j]->p.x - ch[i]->p.x);
            } else {
                angle = atan2(ch[i]->p.y - ch[j]->p.y,ch[i]->p.x - ch[j]->p.x);
            }
        }


        ch[i]->angle = (angle < 0) ? angle + M_PI + M_PI : angle;
        if (angle > 0 && angle < minAngle) {
            minAngle = angle;
            minIdx = i;
        }
    }
    return minIdx;
}

int convexHullQS(ReducedPointPtr *points,int numPoint,ReducedPointPtr *ch)
{
    //find pivot
    int i,j,pvt;
    pvt = 0;
    for (i = 0;i < numPoint;++i) {
        if (points[i]->p.y < points[pvt]->p.y) {
            pvt = i;
        }
    }

    //sort by angle && distance
    ReducedPointPtr tmp;
    tmp = points[0];
    points[0] = points[pvt];
    points[pvt] = tmp;

    doSort(1,numPoint-1,points);

    /*
    {   //debug
        printf("------ after sort --------\n");
        for (i = 0;i < numPoint;i++) {
            printf("points: %f %f %f\n",points[i]->p.x,points[i]->p.y,points[i]->angle);
        }
    }
    */


    //graham
    int numCH = 0;
    ch[numCH++] = points[0];
    ch[numCH++] = points[1];
    for (i = 2;i < numPoint;++i) {
        while (numCH >= 2 && crossPoint(*ch[numCH-2],*ch[numCH-1],*points[i]) <= 0 )  {
            --numCH;
        }
        ch[numCH++] = points[i];
    }


    /*
    {   // debug
        printf("--chull--\n");
        for (i = 0;i < numCH;i++) {
            printf("points: %f %f %f\n",ch[i]->p.x,ch[i]->p.y,ch[i]->angle);
        }
    }
    */

    return numCH;

}



bool DaeHeuristicChecker::is3DPosSpan(mvPoint3d* points,int numPoint)
{
    int n = 0;
    int p = 0;
    int i,j,mkIdx;
    ReducedPoint *reducedPoint = new ReducedPoint[numPoint];
    ReducedPointPtr *pos = new ReducedPointPtr[numPoint];
    ReducedPointPtr *neg = new ReducedPointPtr[numPoint];
    ReducedPointPtr *chPos = new ReducedPointPtr[numPoint];
    ReducedPointPtr *chNeg = new ReducedPointPtr[numPoint];
    ReducedPoint *mkSum = new ReducedPoint[numPoint];  //minkowski sum

    int numCHNeg,numCHPos;
    ReducedPoint rp;  // center of mass
    bool hasResult = false;
    bool result = false;

    //shatter point
    for (i = 0;i < numPoint;++i) {
        if (points[i].z > 0) {
            reducedPoint[i].p.x = points[i].x / points[i].z;
            reducedPoint[i].p.y = points[i].y / points[i].z;
            reducedPoint[i].colorTag = 1;
            pos[p++] = &reducedPoint[i];
        } else if (points[i].z < 0) {
            reducedPoint[i].p.x = points[i].x / points[i].z;
            reducedPoint[i].p.y = points[i].y / points[i].z;
            reducedPoint[i].colorTag = -1;
            neg[n++] = &reducedPoint[i];
        } else {
            double nx = points[i].x;
            double ny = points[i].y;
            double length = sqrt(nx * nx + ny * ny);
            reducedPoint[i].p.x = nx/length;
            reducedPoint[i].p.y = ny/length;
            reducedPoint[i].colorTag = 0;
            pos[p++] = &reducedPoint[i];
        }
    }

    // gen ch of negative
    if (!hasResult && n >= 3) {
        numCHNeg = convexHullQS(neg,n,chNeg);

        // calculate center of mass of positive
        double x = 0;
        double y = 0;
        for (j = 0;j < p ;++j) {
            x = x + pos[j]->p.x;
            y = y + pos[j]->p.y;
        }
        rp.p.x = x / p;
        rp.p.y = y / p;
        rp.colorTag = 1;

        //check inside of the cm of the positive
        bool allInSide = true;
        for (i = 0;i < numCHNeg;i++) {
            j = i+1;
            if (j >= numCHNeg) j = 0;
            double value = crossPoint(*chNeg[i],*chNeg[j],rp);
            if (value <= 0) {
                allInSide = false;
                break;
            }
        }
        if (allInSide) {
            hasResult = true;
            result = true;
        }
    } else {
        numCHNeg = n;
        for (i = 0;i < n;++i) {
            chNeg[i] = neg[i];
        }
    }

    // gen ch of positive
    if (!hasResult && p >= 3) {
        numCHPos = convexHullQS(pos,p,chPos);

        // calculate CM of negative
        double x = 0;
        double y = 0;
        for (j = 0;j < n ;++j) {
            x = x + neg[j]->p.x;
            y = y + neg[j]->p.y;
        }
        ReducedPoint rp;
        rp.p.x = x / n;
        rp.p.y = y / n;
        rp.colorTag = -1;

        //check inside of the cm of negative
        bool allInSide = true;
        for (i = 0;i < numCHPos;i++) {
            j = i+1;
            if (j >= numCHPos) j = 0;

            double value = crossPoint(*chPos[i],*chPos[j],rp);
            if (value <= 0) {
                allInSide = false;
                break;
            }
        }

        if (allInSide) {
            hasResult = true;
            result = true;
        }
    } else {
        if (!hasResult) {
            numCHPos = p;
            for (i = 0;i < p;++i) {
                chPos[i] = pos[i];
            }
        }
    }

    //check intersection of both ch
    if (!hasResult && numCHPos >= 3 && numCHNeg >= 3) {
        //gen minkowski sum
        int negIdx,posIdx,mkIdx,negCount,posCount,mkNum;
        posIdx = findMinAngle(chPos,numCHPos,true);
        negIdx = findMinAngle(chNeg,numCHNeg,false);

        mkIdx = 0;
        negCount = numCHNeg;
        posCount = numCHPos;
        while (negCount > 0 || posCount > 0) {
            //generate mkSum[idx] from chPos[idx] + (- chNeg[negIdx])

            if (chPos[posIdx]->colorTag != 0 && chNeg[negIdx]->colorTag !=0) {
                mkSum[mkIdx].p = chPos[posIdx]->p - chNeg[negIdx]->p;
            } else {
                if (chPos[posIdx]->angle > chNeg[negIdx]->angle) {
                    mkSum[mkIdx].p = chPos[posIdx]->p;
                    mkSum[mkIdx].colorTag = 0;
                } else {
                    mkSum[mkIdx].p = chNeg[negIdx]->p;
                    mkSum[mkIdx].colorTag = 0;
                }
            }
            mkIdx++;
            if (negCount == 0 || (posCount > 0 && chPos[posIdx]->angle < chNeg[negIdx]->angle)) {
                posIdx = (posIdx + 1 == numCHPos) ? 0 : posIdx + 1;
                posCount--;
            } else if (posCount == 0 || (negCount > 0 && chPos[posIdx]->angle > chNeg[negIdx]->angle)) {
                negIdx = (negIdx + 1 == numCHNeg) ? 0 : negIdx + 1;
                negCount--;
            } else {
                posIdx = (posIdx + 1 == numCHPos) ? 0 : posIdx + 1;
                negIdx = (negIdx + 1 == numCHNeg) ? 0 : negIdx + 1;
                posCount--;
                negCount--;
            }
        }
        mkNum = mkIdx;

        //check origin inside
        bool allInside = true;
        int k;
        for (i = 0;i <  mkNum;++i) {
            j = (i + 1 == mkNum) ? 0 : i+1;
            k = (j + 1 == mkNum) ? 0 : j+1;
            if ( crossPoint(mkSum[i],mkSum[j],mkSum[k]) < 0 ) {
                allInside = false;
                break;
            }
        }
        result = allInside;
        hasResult = true;
    }

    //check intersect in the case that either pos or neg is a segment
    if (!hasResult) {

        //check intersect
        if (numCHNeg == 2) {
            for (i = 0;i < numCHPos;i++) {
                j = i+1;
                if (j >= numCHPos) j = 0;
                if (isIntersect( *chPos[i], *chPos[j],
                                 *chNeg[0], *chNeg[1])) {
                    result = true;
                    break;
                }
            }
        } else if (numCHPos == 2) {
            for (i = 0;i < numCHNeg;i++) {
                j = i+1;
                if (j >= numCHNeg) j = 0;
                if (isIntersect( *chNeg[i], *chNeg[j],
                                 *chPos[0], *chPos[1])) {
                        result =  true;
                        break;
                }
            }
        }

        result = false;
    }


    //finalize
    delete [] reducedPoint;
    delete [] pos;
    delete [] neg;
    delete [] chPos;
    delete [] chNeg;
    delete [] mkSum;

    return result;
}

bool DaeHeuristicChecker::is3DPosSpanBrute(mvPoint3d* points,int numPoint)
{
    int n = 0;
    int p = 0;
    int i,j,mkIdx;
    ReducedPoint *reducedPoint = new ReducedPoint[numPoint];
    ReducedPointPtr *pos = new ReducedPointPtr[numPoint];
    ReducedPointPtr *neg = new ReducedPointPtr[numPoint];
    ReducedPointPtr *chPos = new ReducedPointPtr[numPoint];
    ReducedPointPtr *chNeg = new ReducedPointPtr[numPoint];
    ReducedPoint *mkSum = new ReducedPoint[numPoint];  //minkowski sum

    int numCHNeg,numCHPos;
    ReducedPoint rp;  // center of mass
    bool hasResult = false;
    bool result = false;

    //shatter point
    for (i = 0;i < numPoint;++i) {
        if (points[i].z > 0) {
            reducedPoint[i].p.x = points[i].x / points[i].z;
            reducedPoint[i].p.y = points[i].y / points[i].z;
            reducedPoint[i].colorTag = 1;
            pos[p++] = &reducedPoint[i];
        } else if (points[i].z < 0) {
            reducedPoint[i].p.x = points[i].x / points[i].z;
            reducedPoint[i].p.y = points[i].y / points[i].z;
            reducedPoint[i].colorTag = -1;
            neg[n++] = &reducedPoint[i];
        } else {
            double nx = points[i].x;
            double ny = points[i].y;
            reducedPoint[i].p.x = nx;
            reducedPoint[i].p.y = ny;
            reducedPoint[i].colorTag = 0;
            pos[p++] = &reducedPoint[i];
        }
    }

    bool checkPos;             //indicate that we should check whether Positive point lies in negative convex hull
                                // if false, we should check neg inside pos
    int smaller,larger;

    if (p <= n) {
        checkPos = true;
        smaller = p;
        larger = n;
    } else {
        checkPos = false;
        smaller = n;
        larger = p;
    }

    int i1,i2;
    int j1,j2,j3;

    //check inside
    if (n >= 3) {
        for (j1 = 0; j1 < n-2; j1++) {
            for (j2 = j1+1; j2 < n-1; j2++) {
                for (j3 = j2+1; j3 <n; j3++) {
                    //gen convex hull of *neg[j1],*neg[j2],*neg[j3]
                    static int sign;
                    if (crossPoint(*neg[j1],*neg[j2],*neg[j3]) > 0) {
                        //order is c1,c2,c3  sign is positive
                        for (i1 = 0;i1 < p;i1++) {
                            if ((crossPoint(*neg[j1],*neg[j2],*pos[i1]) > 0) &&
                                (crossPoint(*neg[j2],*neg[j3],*pos[i1]) > 0) &&
                                (crossPoint(*neg[j3],*neg[j1],*pos[i1]) > 0)) {
                                    delete [] reducedPoint;
                                    delete [] pos;
                                    delete [] neg;
                                    delete [] chPos;
                                    delete [] chNeg;
                                    delete [] mkSum;
                                    return true;
                            }
                        }
                    } else {
                        //order is c1,c3,c2
                        for (i1 = 0;i1 < p;i1++) {
                            if ((crossPoint(*neg[j1],*neg[j2],*pos[i1]) < 0) &&
                                (crossPoint(*neg[j2],*neg[j3],*pos[i1]) < 0) &&
                                (crossPoint(*neg[j3],*neg[j1],*pos[i1]) < 0)) {
                                    delete [] reducedPoint;
                                    delete [] pos;
                                    delete [] neg;
                                    delete [] chPos;
                                    delete [] chNeg;
                                    delete [] mkSum;
                                    return true;
                            }
                        }
                    }
                }
            }
        }
    }

    if (p >= 3) {
        for (j1 = 0; j1 < p-2; j1++) {
            for (j2 = j1+1; j2 < p-1; j2++) {
                for (j3 = j2+1; j3 <p; j3++) {
                    //gen convex hull of *neg[j1],*neg[j2],*neg[j3]
                    static int sign;
                    if (crossPoint(*pos[j1],*pos[j2],*pos[j3]) > 0) {
                        //order is c1,c2,c3  sign is positive
                        for (i1 = 0;i1 < n;i1++) {
                            if ((crossPoint(*pos[j1],*pos[j2],*neg[i1]) > 0) &&
                                (crossPoint(*pos[j2],*pos[j3],*neg[i1]) > 0) &&
                                (crossPoint(*pos[j3],*pos[j1],*neg[i1]) > 0)) {
                                    delete [] reducedPoint;
                                    delete [] pos;
                                    delete [] neg;
                                    delete [] chPos;
                                    delete [] chNeg;
                                    delete [] mkSum;
                                    return true;
                            }

                        }
                    } else {
                        //order is c1,c3,c2
                        for (i1 = 0;i1 < n;i1++) {
                            if ((crossPoint(*pos[j1],*pos[j2],*neg[i1]) < 0) &&
                                (crossPoint(*pos[j2],*pos[j3],*neg[i1]) < 0) &&
                                (crossPoint(*pos[j3],*pos[j1],*neg[i1]) < 0)) {
                                    delete [] reducedPoint;
                                    delete [] pos;
                                    delete [] neg;
                                    delete [] chPos;
                                    delete [] chNeg;
                                    delete [] mkSum;

                                    return true;
                            }
                        }
                    }
                }
            }
        }
    }

    //check intersect
    for (i1 = 0;i1 < smaller-1;i1++) {
        for (i2 = i1+1;i2 < smaller;i2++) {

            for (j1 = 0; j1 < larger-1; j1++) {
                for (j2 = j1+1; j2 < larger; j2++) {
                    if (checkPos) {
                        if (isIntersect( *pos[i1], *pos[i2],
                                         *neg[j1], *neg[j2])) {
                                        delete [] reducedPoint;
                                        delete [] pos;
                                        delete [] neg;
                                        delete [] chPos;
                                        delete [] chNeg;
                                        delete [] mkSum;
                                        return true;
                        }
                    } else {
                        if (isIntersect( *neg[i1], *neg[i2],
                                         *pos[j1], *pos[j2]))  {
                                        delete [] reducedPoint;
                                        delete [] pos;
                                        delete [] neg;
                                        delete [] chPos;
                                        delete [] chNeg;
                                        delete [] mkSum;
                                        return true;
                        }
                    }
                }
            }
        }
    }

    delete [] reducedPoint;
    delete [] pos;
    delete [] neg;
    delete [] chPos;
    delete [] chNeg;
    delete [] mkSum;

    return false;
}


void DaeHeuristicChecker::test()
{
    srand(0);

    int fp = 0;
    int fn = 0;
    int correct = 0;
    int yes = 0;
    int no = 0;

    mvPoint3d norm,origin(0,0,0),nz(0,0,1);
    mvPoint3d f1,f2,f3;
    double theta,phi;
    double x,y,z;
    GraspingPoint3D gp1,gp2,gp3,gp4;


    for (int i = 0;i < 1;i++) {
        //generate two cone

        f1 = mvPoint3d(-0.893134,0.343733,-0.290101);
        f2 = mvPoint3d(0.015175,0.775773,0.630829);
        f3 = mvPoint3d(0.539252,-0.776700,0.325492);

        //be noted that this is NON-uniform distribution (even though it looks like uniform one)
        theta = double(rand()*M_PI*2)/(1.0+RAND_MAX);
        phi = double(rand()*M_PI*2)/(1.0+RAND_MAX);
        norm.x = cos(theta)*sin(phi);
        norm.y = sin(theta)*sin(phi);
        norm.z = cos(phi);
        gp1 = GraspingPoint3D(origin,norm,halfAngle);

        theta = double(rand()*M_PI*2)/(1.0+RAND_MAX);
        phi = double(rand()*M_PI*2)/(1.0+RAND_MAX);
        norm.x = cos(theta)*sin(phi);
        norm.y = sin(theta)*sin(phi);
        norm.z = cos(phi);
        gp2 = GraspingPoint3D(origin,norm,halfAngle);

        theta = double(rand()*M_PI*2)/(1.0+RAND_MAX);
        phi = double(rand()*M_PI*2)/(1.0+RAND_MAX);
        norm.x = cos(theta)*sin(phi);
        norm.y = sin(theta)*sin(phi);
        norm.z = cos(phi);
        gp3 = GraspingPoint3D(origin,norm,halfAngle);

        theta = double(rand()*M_PI*2)/(1.0+RAND_MAX);
        phi = double(rand()*M_PI*2)/(1.0+RAND_MAX);
        norm.x = cos(theta)*sin(phi);
        norm.y = sin(theta)*sin(phi);
        norm.z = cos(phi);
        gp4 = GraspingPoint3D(origin,norm,halfAngle);

        /*
        gp1.genWrenches(0,0,0,32);
        gp2.genWrenches(0,0,0,32);
        gp3.genWrenches(0,0,0,32);
        gp4.genWrenches(0,0,0,32);
        */

        bool myRes,yhlRes;
        //myRes = isForceConeSpan3(gp1.getNormal(),gp2.getNormal(),gp3.getNormal());
        //yhlRes = isForceConeSpan3Test(gp1,gp2,gp3);

        gp1.genWrenches(0,0,0,32);
        gp2.genWrenches(0,0,0,32);
        gp3.genWrenches(0,0,0,32);
        gp4.genWrenches(0,0,0,32);


        if (myRes != yhlRes) {
            if (myRes) {

                /*
                printf("fP\n");
                printf("%f %f %f\n",gp1.getNormal().x,gp1.getNormal().y,gp1.getNormal().z);
                printf("%f %f %f\n",gp2.getNormal().x,gp2.getNormal().y,gp2.getNormal().z);
                printf("%f %f %f\n",gp3.getNormal().x,gp3.getNormal().y,gp3.getNormal().z);
                fp++;

                gp1 = GraspingPoint3D(origin,f1,halfAngle * 2);
                gp2 = GraspingPoint3D(origin,f2,halfAngle * 2);
                gp3 = GraspingPoint3D(origin,f3,halfAngle * 2);
                gp1.genWrenches(0,0,0,32);
                gp2.genWrenches(0,0,0,32);
                gp3.genWrenches(0,0,0,32);



                printf("fc 1 --------\n");
                gp1.printForceConeGL();
                printf("fc 2 --------\n");
                gp2.printForceConeGL();
                printf("fc 3 --------\n");
                gp3.printForceConeGL();
                */
            }
            if (yhlRes) fn++;
        } else {
            correct++;
            if (myRes) yes++; else no++;
        }

        gp1.clearWrench();
        gp2.clearWrench();
        gp3.clearWrench();
        gp4.clearWrench();


        if (i % 1000 == 0) printf("checked %d\n",i);
    }

    printf("correct %d(yes:%d no:%d) fp %d fn %d\n",correct,yes,no,fp,fn);
}
