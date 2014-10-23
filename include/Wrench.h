#ifndef WRENCH_H
#define WRENCH_H

#include <Eigen/Dense>
#include <tuple>
#include <cmath>

class Wrench : public Eigen::Matrix<double,6,1>
{
    public:
        Wrench();
        virtual ~Wrench();
        Wrench(double fx, double fy, double fz, double tx, double ty, double tz)
        {
            *this << fx, fy, fz, tx, ty, tz;
        }
        Wrench(Eigen::Vector3d position, Eigen::Vector3d force)
        {
            force.normalize();
            Eigen::Vector3d torque = position.cross(force);
            *this << force.x(), force.y(), force.z(), torque.x(), torque.y(), torque.z();
        }

        Eigen::Vector3d force()
        {
            return Eigen::Vector3d((*this)(0), (*this)(1), (*this)(2));
        }

        Eigen::Vector3d torque()
        {
            return Eigen::Vector3d((*this)(3), (*this)(4), (*this)(5));
        }

    protected:
    private:
};

#endif // WRENCH_H
