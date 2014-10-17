#include<vector>
#include<set>
#include<omp.h>
#include<UnitTest++.h>
#include<Qhull.h>
#include<QhullFacetList.h>
#include"Object.hpp"
#include"Grasp.hpp"
#include"RangeTree.hpp"
#include"find_equilibrium_grasps.hpp"

#include"printDebug.hpp"
using namespace std;
// divide each axis into K-1 box
#define K ((1<<4)+1)
// half angle in radian
#define HALFANGLE (M_PI/18)
inline Object readObject(const char* filename){
	int n;
	double px,py,pz,nx,ny,nz;
	vector<PointNormal> points;
	FILE *fp=fopen(filename,"r");
	fscanf(fp,"%d",&n);
	points.reserve(n);
	while(n--){
		fscanf(fp,"%lf%lf%lf%lf%lf%lf",&px,&py,&pz,&nx,&ny,&nz);
		// input normal is pointing outward
		points.push_back(PointNormal(px,py,pz,-nx,-ny,-nz));
	}
	fclose(fp);
	return Object(points);
}

inline vector<pair<PointNormal,Point3d> > filterPoint(const vector<PointNormal> &points, const Point3d &p){
	vector<pair<PointNormal,Point3d> > filtered;
	for(const PointNormal &pn : points){
//		pn.n=p-pn.point;
		Point3d n=(p-pn.point).normalized();
		double angle=pn.normal.angleBetween(n);
		if(angle<HALFANGLE){
//			pn.inverse=false;
			filtered.push_back(pair<PointNormal,Point3d>(pn,n));
		}
		else if((M_PI-HALFANGLE)<angle){
//			pn.inverse=true;
//			pn.n.flipSign();
			filtered.push_back(pair<PointNormal,Point3d>(pn,-n));
		}
	}
	return filtered;
}

inline bool isFC3D(const Grasp &g){
	// test if g is force-closure grasp using Yu Zheng's in python
	char *grasp=print(g);
	FILE *fp=fopen("/tmp/g","w");
	fprintf(fp,"%s",grasp);
	fclose(fp);
	return !system("python isFC3D.py < /tmp/g");
}

int main()
{
	Object object=readObject("/home/qqq/Desktop/spectralMesh/ammo500.txt");
	printf("Object has %ld points\n",object.size());
	set<Grasp> sol;
	//double x,y,z;
	printf("Min: %lf %lf %lf\nMax: %lf %lf %lf\nRange: %lf %lf %lf\n",object.minx,object.miny,object.minz,object.maxx,object.maxy,object.maxz,object.xrange,object.yrange,object.zrange);
	#pragma omp parallel for schedule(static, 1)
	for(int i=1;i<K;++i){
		double x=object.minx+i*object.xrange/K;
		for(int j=1;j<K;++j){
			double y=object.miny+j*object.yrange/K;
			for(int k=1;k<K;++k){
				double z=object.minz+k*object.zrange/K;
				Point3d p(x,y,z);
				vector<pair<PointNormal,Point3d> > M=filterPoint(object,p);
				if(3<M.size()){
					vector<Grasp> subsol=find_equilibrium_grasps(M);
					#pragma omp critical
					sol.insert(subsol.begin(),subsol.end());
				}
			}
		}
	}
	printf("#sol: %ld\n",sol.size());
	return 0;
	// check if solution is FC using Zheng's method
	for(const Grasp &g : sol){
		if(!isFC3D(g))
			printf("%s\n",print(g));
	}
	return 0;
}
