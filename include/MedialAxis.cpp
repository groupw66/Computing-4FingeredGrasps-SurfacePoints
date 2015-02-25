#include "MedialAxis.h"

MedialAxis::MedialAxis()
{
    //ctor
}

MedialAxis::~MedialAxis()
{
    //dtor
}

int MedialAxis::setObject(ObjectSurfacePoints _osp, int nnMode)
{
    positions.clear();
    normals.clear();
    for(auto sp=_osp.surfacePoints.begin() ; sp!= _osp.surfacePoints.end() ; ++sp){
        positions.push_back(sp->position);
        normals.push_back(sp->normal);
    }
    positions.shrink_to_fit();
    normals.shrink_to_fit();
    if(nnMode == 2){
        naiveNN3D.initPoints(positions);
        nn3D = &naiveNN3D;
    }
    else{
        flann3D.initPoints(positions);
        nn3D = &flann3D;
    }

    return 0;
}

double MedialAxis::computeRadius(Eigen::Vector3d pA, Eigen::Vector3d nA, Eigen::Vector3d pB)
{
    Eigen::Vector3d vectorBA = pA-pB;
    double dist = vectorBA.norm();
    return dist*dist / ( 2 * nA.dot(vectorBA) );
}

int MedialAxis::genMedialPoint(std::tuple<Eigen::Vector3d, double, double>& medialPoint, Eigen::Vector3d &pointB,
                               int iA, int iB, int randBMode, double minradius)
{
    std::uniform_int_distribution<int> randInt(0,positions.size()-1);
    std::random_device rd;
    std::default_random_engine rng(rd());
    if(iA < 0){
        iA = randInt(rng);
    }
    while(iB<0 || iB==iA ||
          (randBMode==1 && normals[iA].dot(positions[iA]-positions[iB]) >= 0) ||
          (randBMode==-1 && normals[iA].dot(positions[iA]-positions[iB]) <= 0)
          ){
        iB = randInt(rng);
    }
    if((unsigned int)iA >= positions.size() || (unsigned int)iB >= positions.size()){
        return -1;
    }
    Eigen::Vector3d pA = positions[iA];
    Eigen::Vector3d nA = normals[iA];
    Eigen::Vector3d pB = positions[iB];
    return genMedialPoint(medialPoint, pointB, pA, nA, pB, minradius);
}

int MedialAxis::genMedialPoint(std::tuple<Eigen::Vector3d, double, double>& medialPoint, Eigen::Vector3d &pointB,
                               Eigen::Vector3d pA_, Eigen::Vector3d nA_, Eigen::Vector3d pB_, double minradius)
{
    Eigen::Vector3d pA = pA_;
    Eigen::Vector3d nA = -nA_.normalized();
    Eigen::Vector3d pB = pB_;
    Eigen::Vector3d pM;
    double radius, radius0;
    radius = computeRadius(pA, nA, pB);
    do{
        pM = pA - nA*radius;

        std::vector< std::tuple<Eigen::Vector3d, int, double> > nnPoints;
        std::vector< std::pair<Eigen::Vector3d, double> > nnPoints2;
        nn3D->knnSearch(pM, nnPoints, 2);
        if(std::get<0>(nnPoints[0]) != pA){
            pB = std::get<0>(nnPoints[0]);
        }
        else{
            pB = std::get<0>(nnPoints[1]);
        }

        radius0 = radius;
        radius = computeRadius(pA, nA, pB);
        if(radius < minradius){
            return -1;
        }
    }while(!(radius < radius0+0.000001 && radius > radius0-0.000001));
    medialPoint = std::make_tuple(pM, std::abs(radius), Geometry::angleBetweenVectors(pA-pM, pB-pM));
    pointB = pB;
    return 0;

}

int MedialAxis::genMedialPoints(std::vector< std::tuple<Eigen::Vector3d, double, double> >& medialPoints,
                                int randBMode, double minradius)
{
    medialPoints.clear();
    Eigen::Vector3d pB = positions[1];
    for(unsigned int a=0 ; a<positions.size() ; ++a){
        Eigen::Vector3d pA = positions[a];
        Eigen::Vector3d nA = normals[a];
        Eigen::Vector3d pB_;
        std::tuple<Eigen::Vector3d, double, double> pM;
        if(genMedialPoint(pM, pB_, a, -1, randBMode, minradius)==0){
            medialPoints.push_back(pM);
            pB = pB_;
        }
    }
    return 0;
}
