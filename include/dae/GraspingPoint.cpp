#include "GraspingPoint.h"
#include "Utils.h"

#define EPSILON 1e-10

//==============================================================================
// Grasping Point 3D
//==============================================================================
GraspingPoint3D::GraspingPoint3D(vector3 position,vector3 normal,double halfAngleDegree)
{
    normalWrench = new data6[1];
    wrenches = NULL;
    G = NULL;
    this->position = position;
    this->normal = normal.normalize();
    this->halfangle = halfAngleDegree * M_PI / 180.d;
    this->cosHalfangle = cos(this->halfangle);
    this->sinHalfangle = sin(this->halfangle);
    this->tanHalfangle = tan(this->halfangle);
    this->numWrenches = 0;
}

GraspingPoint3D::GraspingPoint3D()
{
    normalWrench = new data6[1];
    wrenches = NULL;
    G = NULL;
    this->position = vector3();
    this->normal = vector3();
    this->halfangle = 0;
    this->cosHalfangle = cos(halfangle);
    this->sinHalfangle = sin(halfangle);
}


GraspingPoint3D::~GraspingPoint3D()
{
    if (normalWrench != NULL) {
        delete [] normalWrench;
    }
    if (wrenches != NULL) {
        delete [] wrenches;
    }

    if (G != NULL)  {
	    for (int i = 0;i < 6;++i) {
		    free(G[i]);
	    }
	    free(G);
    }

}

void GraspingPoint3D::clearWrench()
{
    if (wrenches != NULL) {
        delete [] wrenches;
    }
    wrenches = NULL;
}

void GraspingPoint3D::genWrenches(GP_DATATYPE ox,GP_DATATYPE oy,GP_DATATYPE oz,int numWrenches) {
    if (wrenches != NULL) {
        delete [] wrenches;
    }
    wrenches = new data6[numWrenches];
    this->numWrenches = numWrenches;

    // position vector
    vector3 r(position.x - ox, position.y - oy, position.z - oz);

    //find one vector perpendicular to the normal vector;
    vector3 planeVect;
    if (normal.z != 0)
        planeVect = vector3(1,1, (0 - normal.x - normal.y) / normal.z);
    else if (normal.y != 0)
        planeVect = vector3(1, (0 - normal.x - normal.z) / normal.y, 1);
    else if (normal.x != 0)
        planeVect = vector3((0 - normal.y - normal.z) / normal.x, 1, 1);


    planeVect = planeVect.normalize();
    //planeVect = GramSchmidt(normal,planeVect);

    //find first vector on the cone
    myQuaternion quatRotateNormal(halfangle, planeVect);
    vector3 rotated = myMatrix44(quatRotateNormal)* normal;

    //generate quaternoin for rotating it around the normal
    float coneSideAngle = 2.0f * M_PI / numWrenches;
    myQuaternion quatRotateConeLine(coneSideAngle, normal);
    myMatrix44 matRotateConeLine(quatRotateConeLine);

    //generate the rest of the cone
    for (int i = 0;i < numWrenches;++i) {
        //leda_vector moment = crossProduct(position,rotated);
        vector3 moment = r * rotated;

        wrenches[i][0] = rotated.x;
        wrenches[i][1] = rotated.y;
        wrenches[i][2] = rotated.z;
        wrenches[i][3] = moment.x;
        wrenches[i][4] = moment.y;
        wrenches[i][5] = moment.z;

        rotated = matRotateConeLine * rotated;
    }
}

void GraspingPoint3D::genNormalWrench(GP_DATATYPE ox,GP_DATATYPE oy,GP_DATATYPE oz) {
    // position vector
    vector3 r(position.x - ox, position.y - oy, position.z - oz);
    vector3 moment = r * normal;
    normalWrench[0][0] = normal.x;
    normalWrench[0][1] = normal.y;
    normalWrench[0][2] = normal.z;
    normalWrench[0][3] = moment.x;
    normalWrench[0][4] = moment.y;
    normalWrench[0][5] = moment.z;
}


void GraspingPoint3D::printForceConeGL(FILE* fp)
{
    fprintf(fp,"glBegin(GL_TRIANGLES);\n");
    int j;
    for (int i = 0;i < numWrenches;++i) {
        j = (i+1 == numWrenches) ? 0:i+1;
        fprintf(fp,"glVertex3f(0,0,0);\n");
        fprintf(fp,"glVertex3f(%f,%f,%f);\n",wrenches[i][0],wrenches[i][1],wrenches[i][2]);
        fprintf(fp,"glVertex3f(%f,%f,%f);\n",wrenches[j][0],wrenches[j][1],wrenches[j][2]);
    }
    fprintf(fp,"glEnd();\n");
}

void GraspingPoint3D::printTorqueConeGL(FILE* fp)
{
    fprintf(fp,"glBegin(GL_TRIANGLES);\n");
    int j;
    for (int i = 0;i < numWrenches;++i) {
        j = (i+1 == numWrenches) ? 0:i+1;

        fprintf(fp,"glVertex3f(0,0,0);\n");
        fprintf(fp,"glVertex3f(%f,%f,%f);\n",wrenches[i][3],wrenches[i][4],wrenches[i][5]);
        fprintf(fp,"glVertex3f(%f,%f,%f);\n",wrenches[j][3],wrenches[j][4],wrenches[j][5]);
    }
    fprintf(fp,"glEnd();\n");
}

void GraspingPoint3D::printTorqueFanGL(FILE* fp)
{
    fprintf(fp,"glBegin(GL_TRIANGLES);\n");
    fprintf(fp,"glVertex3f(0,0,0);\n");
    fprintf(fp,"glVertex3f(%f,%f,%f);\n",torqueFan.v1.x,torqueFan.v1.y,torqueFan.v1.z);
    fprintf(fp,"glVertex3f(%f,%f,%f);\n",torqueFan.v2.x,torqueFan.v2.y,torqueFan.v2.z);
    fprintf(fp,"glEnd();\n");
}

data6* GraspingPoint3D::getWrench(int idx)
{
    if (idx >= 0 && idx < numWrenches) {
        return &wrenches[idx];
    } else return NULL;

}

data6* GraspingPoint3D::getNormalWrench()
{
    if (normalWrench != NULL) {
        return &normalWrench[0];
    } else return NULL;

}


int GraspingPoint3D::getWrenchCount()
{
    return numWrenches;
}

const vector3& GraspingPoint3D::getNormal() {
    return normal;
}

const vector3& GraspingPoint3D::getPosition() {
    return position;
}

#include "Display.h"

void GraspingPoint3D::genTorqueFan(GP_DATATYPE ox,GP_DATATYPE oy,GP_DATATYPE oz)
{
    vector3 newPos(position.x - ox,position.y - oy,position.z - oz);      //new position vector

    //check for the case when Torque span the plane
    double angle = acos((normal ^ newPos)/newPos.getSize());
    if (angle < halfangle || angle > (M_PI - halfangle)) {
        torqueFan.normal = newPos.normalize();
        torqueFan.isPlane = true;
    } else {
        //first, identify the edge vector of the cone
        vector3 leftEdge,rightEdge;
        float dotValue = newPos ^ normal;
        mvMatrix44d rotLeft,rotRight;
        mvPoint3d newPosNorm = newPos.normalize();
        if (fabs(dotValue) <= EPSILON) {
            //special case
            genRot(newPosNorm,rotLeft,cosHalfangle,sinHalfangle);
            leftEdge = rotLeft * normal;
            genRot(newPosNorm,rotRight,cosHalfangle,-sinHalfangle);
            rightEdge = rotRight * normal;
        } else {
            //bool reverseX = dotValue < 0;
            int reverseSign = dotValue < 0? -1 : 1;
            vector3 newPosAdjusted = newPos * reverseSign;

            //calculate the value on the plane perpendicular to normal and contain the point r
            double radius = tanHalfangle * fabs(dotValue);
            vector3 normalAtPlane = normal * fabs(dotValue);
            double p = (newPosAdjusted - normalAtPlane).getSize();

            //calculate the rotation angle for the tangent vector
            double tangentAngle = acos(radius/p);

            /*
            mvPoint3d xaxis = GramSchmidt(normal,newPosAdjusted,true);
            double px = newPosAdjusted ^ xaxis;
            double pz = fabs(dotValue);
            double t = tanHalfangle;
            double tangentAngle = acos(pz*t/px);
            */

            //generate seed vector
            mvMatrix44d rotSeed;
            genRot((newPosAdjusted * normal).normalize(),rotSeed,cos(-halfangle),sin(-halfangle));
            vector3 seed = rotSeed * normal;

            //generate the actual vector by rotation
            //myQuaternion quatLeft(-tangentAngle,normal);
            //myQuaternion quatRight(+tangentAngle,normal);
            genRot(normal,rotLeft,cos(-tangentAngle),sin(-tangentAngle));
            genRot(normal,rotRight,cos(+tangentAngle),sin(+tangentAngle));
            leftEdge = rotLeft * seed;
            rightEdge = rotRight * seed;

            /*
            printf("\nglColor3f(1,0,0);\nglBegin(GL_TRIANGLES);\nglVertex3f(0,0,0);\n");
            printf("glVertex3f(%f,%f,%f);\n",newPos.x,newPos.y,newPos.z);
            printf("glVertex3f(%f,%f,%f);\n",leftEdge.x,leftEdge.y,leftEdge.z);

            printf("\nglColor3f(0,1,0);\nglVertex3f(0,0,0);\n");
            printf("glVertex3f(%f,%f,%f);\n",newPos.x,newPos.y,newPos.z);
            printf("glVertex3f(%f,%f,%f);\n",rightEdge.x,rightEdge.y,rightEdge.z);
            printf("glEnd();\n");
            */

            //check edge vector
            mvPoint3d leftNorm = newPos * leftEdge;
            bool hasPos = false;
            bool hasNeg = false;
            for (int i = 0;i < numWrenches;++i) {
                mvPoint3d force(wrenches[i][0],wrenches[i][1],wrenches[i][2]);
                float sign = force ^ leftNorm;
                if (sign > 0) hasPos = true;
                if (sign < 0) hasNeg = true;
            }

            if (hasPos && hasNeg) {
                printf("error woi!!!\n");
                printf("pos = (%f,%f,%f);\n",newPos.x,newPos.y,newPos.z);
                printf("norm = (%f,%f,%f);\n",normal.x,normal.y,normal.z);
            }

        }
        //calculate the torque vector
        torqueFan.v1 = (newPos * leftEdge);
        torqueFan.v2 = (newPos * rightEdge);
        torqueFan.isPlane = false;
        torqueFan.normal = (torqueFan.v1 * torqueFan.v2);

        ////fan case -- iteration approach
        //torqueFan.isPlane = false;

        //mvPoint3d r;  //rotational axis
        //r = GramSchmidt(normal,newPos);
        //mvMatrix44d rotMat;
        //mvPoint3d forceEdge1;
        //mvPoint3d forceEdge2;

        //genRot(r,rotMat,cos(+halfangle),sin(+halfangle));
        //forceEdge2 = rotMat * normal;
        //genRot(r,rotMat,cos(-halfangle),sin(-halfangle));
        //forceEdge1 = rotMat * normal;

        ////identify the edge vector

        //int maxStep = 360*1024; //divide 360 degree into maxStep
        //double stepSize = 2 * M_PI / maxStep;
        //mvMatrix44d leftRot;
        //mvMatrix44d rightRot;

        //if ((normal ^ newPos) > 0) {
        //    genRot(normal,leftRot,cos(+stepSize),sin(+stepSize));
        //} else {
        //    genRot(normal,leftRot,cos(-stepSize),sin(-stepSize));
        //}

        //mvPoint3d curr,next;
        //mvPoint3d currNorm;
        //int stepCount;
        //int maxCount = maxStep >> 2;

        ////find left
        //next = forceEdge2;
        //stepCount = 1;
        //do {
        //    /*
        //    curr = next;
        //    next = leftRot * curr;
        //    currNorm = curr * newPos;
        //    */
        //    if ((normal ^ newPos) > 0) {
        //        genRot(normal,leftRot,cos(+stepSize*(stepCount-1)),sin(+stepSize*(stepCount-1)));
        //    } else {
        //        genRot(normal,leftRot,cos(-stepSize*(stepCount-1)),sin(-stepSize*(stepCount-1)));
        //    }
        //    curr = leftRot * forceEdge2;
        //    if ((normal ^ newPos) > 0) {
        //        genRot(normal,leftRot,cos(+stepSize*(stepCount)),sin(+stepSize*(stepCount)));
        //    } else {
        //        genRot(normal,leftRot,cos(-stepSize*(stepCount)),sin(-stepSize*(stepCount)));
        //    }
        //    next = leftRot * forceEdge2;
        //    currNorm = curr * newPos;

        //    stepCount++;
        //} while ((currNorm ^ next) < 0 && stepCount < maxCount);
        //leftEdge = curr;
        //torqueFan.v1 = (newPos * curr).normalize();

        ////find right
        //stepCount--;
        //if ((normal ^ newPos) > 0) {
        //    genRot(normal,rightRot,cos(-stepSize*stepCount),sin(-stepSize*stepCount));
        //} else {
        //    genRot(normal,rightRot,cos(+stepSize*stepCount),sin(+stepSize*stepCount));
        //}
        //curr = rightRot * forceEdge1;
        //rightEdge = curr;
        //torqueFan.v2 = (newPos * curr).normalize();

        ////torqueFan.v2 = crossProduct(p,forceEdge2).norm();
        //torqueFan.normal = torqueFan.v1 * torqueFan.v2;

        //show output
        //SchemeOutput scm("tf2.scm");
        //if (torqueFan.isPlane) {
        //} else {
        //    scm.print_cone(mvPoint3d(0,0,0),normal,100,halfangle);
        //    scm.print_line(mvPoint3d(0,0,0),newPos);
        //    scm.print_line(mvPoint3d(0,0,0),torqueFan.v1);
        //    scm.print_line(mvPoint3d(0,0,0),torqueFan.v2);
        //    scm.setColor(255,0,0);
        //    scm.print_line(mvPoint3d(0,0,0),leftEdge*100);
        //    scm.print_line(mvPoint3d(0,0,0),rightEdge*100);
        //    scm.setColor(0,0,255);
        //    scm.print_plane(mvPoint3d(0,0,0),leftEdge * newPos);
        //    scm.print_plane(mvPoint3d(0,0,0),newPos * leftEdge);
        //    scm.print_plane(mvPoint3d(0,0,0),rightEdge * newPos);
        //    scm.print_plane(mvPoint3d(0,0,0),newPos * rightEdge);
        //}
        //scm.close();


    }



}

void GraspingPoint3D::genLocalFrame()
{
	//generate the local frame

    //we already have the norm from the constructor

	mvPoint3d xAxis(1,0,0);
	mvPoint3d yAxis(0,1,0);

	if ( (normal ^ xAxis) != 1.0) {
		//use xAxis
		double u = normal ^ xAxis;
		tangent1 = (xAxis - normal * u).normalize();
		tangent2 = normal * tangent1;
	} else {
		//use yAxis
		double u = normal ^ yAxis;
		tangent1 = (yAxis - normal * u).normalize();
		tangent2 = normal * tangent1;
	}

    //deallocate G
    if (G != NULL) {
	    for (int i = 0;i < 6;++i) {
		    free(G[i]);
	    }
        free(G);
    }

	//prepare G
	G = (GP_DATATYPE**) malloc(sizeof(GP_DATATYPE*)*6);
	for (int i = 0;i < 6;++i) {
		G[i] = (GP_DATATYPE*) malloc(sizeof(GP_DATATYPE) * 3);
	}

	//put the top part of Gu with the rotation matrix that tranform the local frame to the global frame (R^(-1))
	// R^(-1) = R^T = [ tangent1 tangent2 norm] (where each is a column vector)
	G[0][0] = tangent1.x; G[1][0] = tangent1.y; G[2][0] = tangent1.z;
	G[0][1] = tangent2.x; G[1][1] = tangent2.y; G[2][1] = tangent2.z;
	G[0][2] = normal.x;   G[1][2] = normal.y;   G[2][2] = normal.z;

	//for the lower part Gl = TR^(-1)x, where T is the tranformation that convert x to the torque of x

	//first, generate the row vector of T
	mvPoint3d r1( 0,-position.z, position.y);
	mvPoint3d r2( position.z, 0,-position.x);
	mvPoint3d r3(-position.y, position.x, 0);

	//do the multiplication TR^(-1)
	G[3][0] = tangent1 ^ r1;    G[3][1] = tangent2 ^ r1;    G[3][2] = normal ^ r1;
	G[4][0] = tangent1 ^ r2;    G[4][1] = tangent2 ^ r2;    G[4][2] = normal ^ r2;
	G[5][0] = tangent1 ^ r3;    G[5][1] = tangent2 ^ r3;    G[5][2] = normal ^ r3;
}

void GraspingPoint3D::getG(GP_DATATYPE **dest,int offset)
{
    for (int i = 0;i < 6;++i) {
        memcpy(&(dest[i][offset]),G[i],sizeof(GP_DATATYPE) * 3);
    }
}

double GraspingPoint3D::getHalfAngle()
{
    return gHalfangle;
}

//==============================================================================
// Grasping Point 2D
//==============================================================================

GraspingPoint2D::GraspingPoint2D(vector2 position,vector2 normal,double halfAngle)
{
    this->normal = normal.normalize();
    this->position = position;

    halfangle = halfAngle;
    sinHalfangle = sin(halfAngle);
    cosHalfangle = cos(halfAngle);

    angle = atan2(normal.y,normal.x);

    leftForce = vector2(
		+ cosHalfangle*normal.x - sinHalfangle*normal.y,
		+ sinHalfangle*normal.x + cosHalfangle*normal.y
	).normalize();
    rightForce = vector2(
		+ cosHalfangle*normal.x + sinHalfangle*normal.y,
		- sinHalfangle*normal.x + cosHalfangle*normal.y
	).normalize();
    genWrenches();
}

GraspingPoint2D::GraspingPoint2D() {
    this->normal = vector2();
    this->position = vector2();
    halfangle = 0;
    sinHalfangle = sin(halfangle);
    cosHalfangle = cos(halfangle);
}

GraspingPoint2D::~GraspingPoint2D() {
}

void GraspingPoint2D::genFan(GP_DATATYPE ox,GP_DATATYPE oy) {

    /* Attawith's Dual
    leda_vector r(x - origin.xcoord(),y - origin.ycoord());

	leda_vector m1(-leftCone.ycoord(), leftCone.xcoord());
    leda_vector m2(-rightCone.ycoord(), rightCone.xcoord());

    dp1 = leda_point(m1/(m1 * r));
    dp2 = leda_point(m2/(m2 * r));

    dualSegm = leda_segment(dp1,dp2);
	dualSign = (normal.xcoord() * r.ycoord() - normal.ycoord() * r.xcoord( )) > 0;
    */

    //leda_point o2(0,0,0);

	float rx = position.x - ox;
	float ry = position.y - oy;


	float torque1 = rx * leftForce.y - ry * leftForce.x;
	float torque2 = rx * rightForce.y - ry * rightForce.x;

    if (torque1 != 0)
        fan.p1 = vector2(leftForce.x / torque1,leftForce.y / torque1);
    else
        fan.p1 = vector2(0,0);

    if (torque2 != 0)
        fan.p2 = vector2(rightForce.x / torque2,rightForce.y / torque2);
    else
        fan.p2 = vector2(0,0);



	fan.hasPosRay = false;
	fan.hasNegRay = false;
	fan.hasSeg = false;
	if (torque1 * torque2 > 0) {
		//dual as segment
		fan.hasSeg = true;
        fan.segIsPositive = (rx * normal.y - ry * normal.x) > 0;
	} else {
		//dual as rays
		if (torque1 > 0) {
			fan.hasPosRay = true;
            fan.posRayPoint = fan.p1;
            fan.posRayDirection = (fan.p1 - fan.p2);
		} else if (torque1 < 0) {
			fan.hasNegRay = true;
            fan.negRayPoint = fan.p1;
            fan.negRayDirection = (fan.p1 - fan.p2);
        } else {
            fan.zeroRay[fan.zeroRayCount] = vector2(leftForce.x,leftForce.y);
            fan.zeroRayCount++;
        }

		if (torque2 > 0) {
			fan.hasPosRay = true;
            fan.posRayPoint = fan.p2;
            fan.posRayDirection = (fan.p2 - fan.p1);
		} else if (torque2 < 0) {
			fan.hasNegRay = true;
            fan.negRayPoint = fan.p2;
            fan.negRayDirection = (fan.p2 - fan.p1);
        } else {
            fan.zeroRay[fan.zeroRayCount] = vector2(rightForce.x,rightForce.y);
            fan.zeroRayCount++;
        }
	}

}

void GraspingPoint2D::genWrenches(GP_DATATYPE ox,GP_DATATYPE oy) {
    leftWrench.x = leftForce.x;
    leftWrench.y = leftForce.y;
    leftWrench.z = (position.x - ox) * leftForce.y - (position.y - oy) * leftForce.x;

    rightWrench.x = rightForce.x;
    rightWrench.y = rightForce.y;
    rightWrench.z = (position.x - ox) * rightForce.y - (position.y - oy) * rightForce.x;
}

vector3* GraspingPoint2D::getWrench(int idx)
{
    if (idx == 0) {
        return &leftWrench;
    } else if (idx == 1) {
        return &rightWrench;
    } else return NULL;

}

vector2 GraspingPoint2D::getNormal()
{
    return normal;
}

vector2 GraspingPoint2D::getPosition()
{
    return position;
}

bool TorqueFan::crossPlane(vector3 plane)
{
    vector3 cross = plane * normal;
    return (cross.x != 0 || cross.y != 0 || cross.z != 0);
}

bool TorqueFan::crossPlane(TorqueFan fan)
{
    if (fan.isPlane) return crossPlane(fan.normal);

    float sign1 = normal ^ fan.v1;
    float sign2 = normal ^ fan.v2;

    return (sign1 * sign2 < 0);
}


