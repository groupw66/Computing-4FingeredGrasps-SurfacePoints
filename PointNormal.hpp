#ifndef POINTNORMAL_HPP_INCLUDED
#define POINTNORMAL_HPP_INCLUDED

#include<utility>
#include"Point3d.hpp"

#define point first
#define normal second
struct PointNormal : std::pair<Point3d, Point3d>{
//	Point3d point,normal;
//	bool inverse;
//	Point3d n; // not used in equality checking
//	PointNormal(double px,double py,double pz,double nx,double ny,double nz):point(px,py,pz),normal(nx,ny,nz){}
	PointNormal(double px,double py,double pz,double nx,double ny,double nz) :
	std::pair<Point3d, Point3d>(Point3d(px,py,pz), Point3d(nx,ny,nz)){
//		std::tie(point,normal)=*this;
	}

//	bool operator <(const PointNormal &b)const{
//		return (point<b.point) || (point==b.point && normal<b.normal);
//	}

//	bool operator ==(const PointNormal &b)const{
//		return !(*this<b || b<*this);
//	}

//	PointNormal(const Point3d &_point,const Point3d &_normal):point(_point),normal(_normal){
//		normal*=1./norm(normal);// input already normalize
//	}
//	double operator - (const PointNormal &z) const{
//		int m=PYRAMIDSIDE;
//		double min=DBL_MAX;
//		for(int x=0;x<m;++x){
//			double sum=0;
//			for(int k=0;k<m;++k) sum+=wrenchs[k]-z.wrenchs[(k+x)%m];
//			if(sum<min) min=sum;
//		}
//		return min/m;
//	}
//	void computeWrench(const Point3d &cm){
//		Point3d ran;
//		if(fabs(normal.x)<1e-6){
//			ran.x=0;
//			ran.y=-normal.z;
//			ran.z=normal.y;
//		}
//		else if(fabs(normal.y)<1e-6){
//			ran.x=-normal.z;
//			ran.y=0;
//			ran.z=normal.x;
//		}
//		else{
//			ran.x=-normal.y;
//			ran.y=normal.x;
//			ran.z=0;
//		}
//		ran*=MU/norm(ran);
////printf("ran=\t%lf %lf %lf\n",ran.x,ran.y,ran.z);
//
////		ran=normal+ran;
//
////printf("newPoint=\t%lf %lf %lf\n",ran.x,ran.y,ran.z);
//		wrenchs.clear();
//		wrenchs.reserve(PYRAMIDSIDE);
//
////		wrenchs.push_back(Wrench(ran,point-cm));
//		Point3d edge=normal+ran;
//		wrenchs.push_back(Wrench(edge,point-cm));
//
////printf("firstWrench=%lf %lf %lf\n",wrenchs[0].w(0),wrenchs[0].w(1),wrenchs[0].w(2));
//		Point3d ranOriginal(ran);
//		const double cosine=cos(ANGLE),sine=sin(ANGLE),sqrtxxyy=sqrt(normal.x*normal.x+normal.y*normal.y);
//		Matx33d rz(cosine,-sine,0,sine,cosine,0,0,0,1),
//			rzi(cosine,sine,0,-sine,cosine,0,0,0,1),
//			tz(normal.z,0,-sqrtxxyy,0,1,0,sqrtxxyy,0,normal.z),
//			txz(normal.x/sqrtxxyy,normal.y/sqrtxxyy,0,-normal.y/sqrtxxyy,normal.x/sqrtxxyy,0,0,0,1),
////			front=txz.inv()*tz.inv(),
//			back=tz*txz,
//			front=back.inv(),
//			r=front*rz*back;
////cout << r << endl;
///*
//		r=rotation_matrix(normal,ANGLE);
//		for(int j=1;j<PYRAMIDSIDE;++j){
//			// rotate ran ccw across normal
//			ran=r*ran;
//			edge=normal+ran;
//			wrenchs.insert(wrenchs.begin(),Wrench(edge,point-cm));
//		}
///*/
//		for(int j=PYRAMIDSIDE&1?0:1;j<PYRAMIDSIDE>>1;++j){
//			// rotate ran ccw across normal
//			ran=r*ran;
//			edge=normal+ran;
//			wrenchs.insert(wrenchs.begin(),Wrench(edge,point-cm));
//		}
//		ran=ranOriginal;
//		r=front*rzi*back;
//		for(int j=0;j<PYRAMIDSIDE>>1;++j){
//			// rotate ran cw across normal
//			ran=r*ran;
//			edge=normal+ran;
//			wrenchs.push_back(Wrench(edge,point-cm));
//		}
//*/
//	}
};

#endif // POINTNORMAL_HPP_INCLUDED
