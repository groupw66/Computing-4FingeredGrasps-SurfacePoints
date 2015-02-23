#include "MedialAxis.h"

MedialAxis::MedialAxis()
{
    //ctor
}

MedialAxis::~MedialAxis()
{
    //dtor
}

MedialAxis::MedialAxis(ObjectSurfacePoints _osp)
{
    setObject(_osp);
}

int MedialAxis::setObject(ObjectSurfacePoints _osp)
{
    positions.clear();
    normals.clear();
    for(auto sp=_osp.surfacePoints.begin() ; sp!= _osp.surfacePoints.end() ; ++sp){
        positions.push_back(sp->position);
        normals.push_back(sp->normal);
    }
    positions.shrink_to_fit();
    normals.shrink_to_fit();
    flann3D.initPoints(positions);
    return 0;
}

double MedialAxis::computeRadius(Eigen::Vector3d pA, Eigen::Vector3d nA, Eigen::Vector3d pB)
{
    Eigen::Vector3d vectorBA = pA-pB;
    double dist = vectorBA.norm();
    return dist*dist / ( 2 * nA.dot(vectorBA) );
}

int MedialAxis::genMedialPoint(std::tuple<Eigen::Vector3d, double, double>& medialPoint, Eigen::Vector3d &pointB, int iA, int iB)
{
    std::uniform_int_distribution<int> randInt(0,positions.size()-1);
    std::random_device rd;
    std::default_random_engine rng(rd());
    if(iA < 0){
        iA = randInt(rng);
    }
    if(iB < 0){
        iB = iA;
        while(iA == iB)
            iB = randInt(rng);
    }
    if(iA >= positions.size() || iB >= positions.size()){
        return -1;
    }
    Eigen::Vector3d pA = positions[iA];
    Eigen::Vector3d nA = normals[iA];
    Eigen::Vector3d pB = positions[iB];
    return genMedialPoint(medialPoint, pointB, pA, nA, pB);
}

int MedialAxis::genMedialPoint(std::tuple<Eigen::Vector3d, double, double>& medialPoint, Eigen::Vector3d &pointB, Eigen::Vector3d pA_, Eigen::Vector3d nA_, Eigen::Vector3d pB_)
{
    Eigen::Vector3d pA = pA_;
    Eigen::Vector3d nA = nA_.normalized();
    Eigen::Vector3d pB = pB_;
    Eigen::Vector3d pM;
    double radius, radius0;
    radius = computeRadius(pA, nA, pB);
    do{
        pM = pA - nA*radius;

        std::vector< std::pair<Eigen::Vector3d, double> > nnPoints;
        flann3D.knnSearch(pM, nnPoints, 2);
        double dist;
        if(nnPoints[0].first != pA){
            pB = nnPoints[0].first;
            dist = nnPoints[0].second;
        }
        else{
            pB = nnPoints[1].first;
            dist = nnPoints[1].second;
        }

        radius0 = radius;
        radius = computeRadius(pA, nA, pB);
    }while(!(radius < radius0+0.000001 && radius > radius0-0.000001));
    medialPoint = std::make_tuple(pM, std::abs(radius), Geometry::angleBetweenVectors(pA-pM, pB-pM));
    pointB = pB;
    return 0;

}

int MedialAxis::genMedialPoints(std::vector< std::tuple<Eigen::Vector3d, double, double> >& medialPoints)
{
    medialPoints.clear();
    Eigen::Vector3d pB = positions[1];
    for(unsigned int a=0 ; a<positions.size() ; ++a){
        Eigen::Vector3d pA = positions[a];
        Eigen::Vector3d nA = normals[a];
        Eigen::Vector3d pB_;
        std::tuple<Eigen::Vector3d, double, double> pM;
        if(genMedialPoint(pM, pB_, pA, nA, pB)==0){
            medialPoints.push_back(pM);
            pB = pB_;
        }
    }
    return 0;
}
