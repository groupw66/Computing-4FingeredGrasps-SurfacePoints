#ifndef SURFACEPOINT_H
#define SURFACEPOINT_H

#include <Eigen/Dense>

class SurfacePoint
{
    public:
        SurfacePoint();
        virtual ~SurfacePoint();
        SurfacePoint(Eigen::Vector3d _position, Eigen::Vector3d _normal);

        Eigen::Vector3d position;
        Eigen::Vector3d normal;
    protected:
    private:
};

#endif // SURFACEPOINT_H
