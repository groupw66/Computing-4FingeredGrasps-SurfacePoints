#ifndef POINT3D_HPP_INCLUDED
#define POINT3D_HPP_INCLUDED

#include<tuple>
#include<cmath>
#include"Point2d.hpp"

struct Point3d : std::tuple<double,double,double>{

	Point3d(){}
	Point3d(double _x,double _y,double _z) :
	std::tuple<double,double,double>(_x,_y,_z){}

	Point3d operator -()const{
		return Point3d(-std::get<0>(*this), -std::get<1>(*this), -std::get<2>(*this));
	}

	Point3d operator +(const Point3d &b)const{
		return Point3d(std::get<0>(*this)+std::get<0>(b), std::get<1>(*this)+std::get<1>(b), std::get<2>(*this)+std::get<2>(b));
	}

	Point3d operator -(const Point3d &b)const{
		return *this+(-b);
	}

	Point3d operator /(const double d)const{
		return Point3d(std::get<0>(*this)/d, std::get<1>(*this)/d, std::get<2>(*this)/d);
	}

	inline double x()const{
		return std::get<0>(*this);
	}

	inline double y()const{
		return std::get<1>(*this);
	}

	inline double z()const{
		return std::get<2>(*this);
	}

	inline double dot(const Point3d &b)const{
		return std::get<0>(*this)*std::get<0>(b) + std::get<1>(*this)*std::get<1>(b) + std::get<2>(*this)*std::get<2>(b);
	}

	inline Point3d cross(const Point3d &b)const{
		return Point3d(
			std::get<1>(*this)*std::get<2>(b) - std::get<2>(*this)*std::get<1>(b),
			std::get<2>(*this)*std::get<0>(b) - std::get<0>(*this)*std::get<2>(b),
			std::get<0>(*this)*std::get<1>(b) - std::get<1>(*this)*std::get<0>(b)
		);
	}

	inline Point3d rotate(const Point3d &Tx,const Point3d &Ty,const Point3d &Tz)const{
		return Point3d(
			this->dot(Tx),
			this->dot(Ty),
			this->dot(Tz)
		);
	}

	inline double norm2()const{
		return sqrt(this->dot(*this));
	}

	inline double angleBetween(const Point3d &b)const{
		return acos(this->dot(b) / this->norm2() / b.norm2());
	}

	inline Point3d normalized()const{
		return *this/this->norm2();
	}

	// to spherical coordinate
	inline Point2d toSph()const{
		return Point2d(atan2(std::get<1>(*this), std::get<0>(*this)), atan2(std::get<2>(*this), sqrt(std::get<0>(*this)*std::get<0>(*this) + std::get<1>(*this)*std::get<1>(*this))));
	}

	// to force dual representation
	inline Point2d toFDR()const{
		return
		std::get<2>(*this)==0?
			Point2d(std::get<0>(*this), std::get<1>(*this))
		:
			Point2d(std::get<0>(*this)/std::get<2>(*this), std::get<1>(*this)/std::get<2>(*this));
	}

	// to force dual representation angle (alpha, beta)
	inline Point2d toFDRAngle(double fanHalfLen)const{
		Point2d fdr=this->toFDR();
		fanHalfLen=(std::get<2>(*this)==0)?0:fanHalfLen;
//		double dya=fanHalfLen-fdr.second,dyb=fanHalfLen+fdr.second;
//		return Point2d(acos(dya/hypot(fdr.first,dya)), acos(dyb/hypot(fdr.first,dyb)));
		double x=fabs(fdr.first);
		return Point2d(atan2(x,fanHalfLen-fdr.second), atan2(x,fdr.second+fanHalfLen));
//auto a=Point2d(acos(dya/hypot(fdr.first,dya)), acos(dyb/hypot(fdr.first,dyb))),
//b=Point2d(atan2(x,fanHalfLen-fdr.second), atan2(x,fdr.second+fanHalfLen));
//double err=fabs(hypot(a.first-b.first,a.second-b.second));
//if(err>1e-9){
//printf("%lf %lf | %lf %lf\n",a.first,a.second,dya/hypot(fdr.first,dya),dyb/hypot(fdr.first,dyb));
//printf("\t%lf %lf | %lf %lf %lf\n",b.first,b.second,x,fanHalfLen,fdr.second);
//}
	}

//	inline void flipSign(){
//		std::get<0>(*this)=-std::get<0>(*this);
//		std::get<1>(*this)=-std::get<1>(*this);
//		std::get<2>(*this)=-std::get<2>(*this);
//	}

//	bool operator <(const Point3d &b)const{
//		return (x<b.x) || (x==b.x && (y<b.y || (y==b.y && z<b.z)));
//	}

//	bool operator ==(const Point3d &b)const{
//		return !(*this<b || b<*this);
//	}
};

#endif // POINT3D_HPP_INCLUDED
