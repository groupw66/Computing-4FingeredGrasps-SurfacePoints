#ifndef OBJECTSURFACEPOINTS_H
#define OBJECTSURFACEPOINTS_H

#include <Eigen/Dense>
#include <vector>
#include <limits>
#include "SurfacePoint.h"
#include "OBJFile.h"
#include "PositionsNormalsFile.h"

class ObjectSurfacePoints
{
    public:
        ObjectSurfacePoints();
        virtual ~ObjectSurfacePoints();
        ObjectSurfacePoints(const OBJFile &_objFile){ open(_objFile); }
        ObjectSurfacePoints(const PositionsNormalsFile &_positionsNormalsFile){ open(_positionsNormalsFile); }

        void open(const OBJFile &_objFile);
        void open(const PositionsNormalsFile &_positionsNormalsFile);
        void computeCM();

        std::vector<SurfacePoint> surfacePoints;
        Eigen::Vector3d cm;
        Eigen::Vector3d minAABB;
        Eigen::Vector3d maxAABB;

    protected:
    private:
};

#endif // OBJECTSURFACEPOINTS_H
