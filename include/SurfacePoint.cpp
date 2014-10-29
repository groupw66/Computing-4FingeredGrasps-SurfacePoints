#include "SurfacePoint.h"

SurfacePoint::SurfacePoint()
{
    //ctor
}

SurfacePoint::~SurfacePoint()
{
    //dtor
}

std::vector<Eigen::Vector3d> SurfacePoint::getFrictionCone(double halfAngle, int nPyramidSide)
{
    std::vector<Eigen::Vector3d> frictionCone;
    frictionCone.resize(nPyramidSide);

    double uFriction = std::tan(halfAngle*M_PI/180.d);

    // find perpendicular vector
    Eigen::Vector3d perpendicular;
    if(normal.x() ==0 ){
      perpendicular = Eigen::Vector3d(0,normal.z(),-normal.y());
    }
    else if(normal.y() ==0 ){
      perpendicular = Eigen::Vector3d(normal.z(),0,-normal.x());
    }
    else{
      perpendicular = Eigen::Vector3d(normal.y(),-normal.x(),0);
    }
    perpendicular.normalize();

    // find some vector on the friction cone
    frictionCone[0] = (normal + perpendicular*uFriction);
    //frictionCone[0].normalize();

    // find all vector on the friction cone
    double rotateStep = (2.d*M_PI)/nPyramidSide;
    double _rotate=0.0 ;
    for(int i=0; i<nPyramidSide ; i++){
      Eigen::Matrix3d rMatrix;
      rMatrix = Eigen::AngleAxisd(_rotate,normal);
      frictionCone[i] = (rMatrix*frictionCone[0]);
      _rotate+=rotateStep;
//      frictionCone[i].normalize();
    }
    return frictionCone;
}

std::vector<Wrench> SurfacePoint::getWrenchCone(Eigen::Vector3d cm, double halfAngle, int nPyramidSide)
{
    std::vector<Eigen::Vector3d> frictionCone = getFrictionCone(halfAngle, nPyramidSide);
    std::vector<Wrench> wrenchCone;
    Eigen::Vector3d p = position - cm;
    for(auto force : frictionCone){
        wrenchCone.push_back(Wrench(p, force));
    }
    return wrenchCone;
}




