#include "ObjectSurfacePoints.h"

ObjectSurfacePoints::ObjectSurfacePoints()
{
    //ctor
}

ObjectSurfacePoints::~ObjectSurfacePoints()
{
    //dtor
}

ObjectSurfacePoints::ObjectSurfacePoints(const OBJFile &_objFile)
{
    surfacePoints.clear();
    minAABB = Eigen::Vector3d(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    maxAABB = Eigen::Vector3d(std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min());
    cm = Eigen::Vector3d(0,0,0);
    for(Eigen::Vector3i facet : _objFile.facets){
        int i=facet.x(), j=facet.y(), k=facet.z();
        Eigen::Vector3d position = (_objFile.vertexs[i] + _objFile.vertexs[j] + _objFile.vertexs[k])/3.0;
        Eigen::Vector3d va = _objFile.vertexs[i] - _objFile.vertexs[j];
        Eigen::Vector3d vb = _objFile.vertexs[k] - _objFile.vertexs[j];
        Eigen::Vector3d normal = va.cross(vb); //direction inward to object
        surfacePoints.push_back(SurfacePoint(position, normal));

        cm += position;

        minAABB.x() = std::min(minAABB.x(), position.x());
        minAABB.y() = std::min(minAABB.y(), position.y());
        minAABB.z() = std::min(minAABB.z(), position.z());
        maxAABB.x() = std::max(maxAABB.x(), position.x());
        maxAABB.y() = std::max(maxAABB.y(), position.y());
        maxAABB.z() = std::max(maxAABB.z(), position.z());

    }
    cm /= surfacePoints.size();
}

ObjectSurfacePoints::ObjectSurfacePoints(const PositionsNormalsFile &_positionsNormalsFile)
{
    surfacePoints.clear();
    minAABB = Eigen::Vector3d(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    maxAABB = Eigen::Vector3d(std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min());
    cm = Eigen::Vector3d(0,0,0);
    for(unsigned int i=0 ; i<_positionsNormalsFile.positions.size() ; ++i){
        Eigen::Vector3d position = _positionsNormalsFile.positions[i];
        Eigen::Vector3d normal = _positionsNormalsFile.normals[i];
        surfacePoints.push_back(SurfacePoint(position, normal));

        cm += position;

        minAABB.x() = std::min(minAABB.x(), position.x());
        minAABB.y() = std::min(minAABB.y(), position.y());
        minAABB.z() = std::min(minAABB.z(), position.z());
        maxAABB.x() = std::max(maxAABB.x(), position.x());
        maxAABB.y() = std::max(maxAABB.y(), position.y());
        maxAABB.z() = std::max(maxAABB.z(), position.z());
    }
    cm /= surfacePoints.size();
}
