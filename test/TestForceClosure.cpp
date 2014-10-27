#include <UnitTest++.h>
#include <iostream>
#include <Eigen/Dense>
#include "ForceClosure.h"
#include "PositionsNormalsFile.h"
#include "ObjectSurfacePoints.h"
#include "Timer.h"
namespace
{

TEST(ForceClosure_getMindist_Qhull)
{
    //TO-DO
}

TEST(ForceClosure_near_Qhull_ZC)
{
	PositionsNormalsFile obj("test/meshes/spectralMesh/ammo50.txt");
	ObjectSurfacePoints osp(obj);
	unsigned int nSurfacePoint = osp.surfacePoints.size();
	int step=1<<3;
	Timer tmr;
	double qHullRunTime=0.,ZCRunTime=0.;
	for(unsigned int a=0 ; a<nSurfacePoint ; a+=step){
		for(unsigned int b=a+1 ; b<nSurfacePoint ; b+=step){
//			printf("SurfacePointId: %d\n",b);
			for(unsigned int c=b+1 ; c<nSurfacePoint ; c+=step){
				for(unsigned int d=c+1 ; d<nSurfacePoint ; d+=step){
//					printf("%.9lf %.9lf %.9lf %.9lf %.9lf %.9lf\n",osp.surfacePoints[a].position.x(),osp.surfacePoints[a].position.y(),osp.surfacePoints[a].position.z(),osp.surfacePoints[a].normal.x(),osp.surfacePoints[a].normal.y(),osp.surfacePoints[a].normal.z());
//					printf("%.9lf %.9lf %.9lf %.9lf %.9lf %.9lf\n",osp.surfacePoints[b].position.x(),osp.surfacePoints[b].position.y(),osp.surfacePoints[b].position.z(),osp.surfacePoints[b].normal.x(),osp.surfacePoints[b].normal.y(),osp.surfacePoints[b].normal.z());
//					printf("%.9lf %.9lf %.9lf %.9lf %.9lf %.9lf\n",osp.surfacePoints[c].position.x(),osp.surfacePoints[c].position.y(),osp.surfacePoints[c].position.z(),osp.surfacePoints[c].normal.x(),osp.surfacePoints[c].normal.y(),osp.surfacePoints[c].normal.z());
//					printf("%.9lf %.9lf %.9lf %.9lf %.9lf %.9lf\n",osp.surfacePoints[d].position.x(),osp.surfacePoints[d].position.y(),osp.surfacePoints[d].position.z(),osp.surfacePoints[d].normal.x(),osp.surfacePoints[d].normal.y(),osp.surfacePoints[d].normal.z());

					tmr.reset();
					double mindistQhull = ForceClosure::getMindist_Qhull(osp.surfacePoints[a], osp.surfacePoints[b], osp.surfacePoints[c], osp.surfacePoints[d], osp.cm);
					qHullRunTime+=tmr.elapsed();
					bool isFCQhull = mindistQhull > 0;

					tmr.reset();
					double mindistZC = ForceClosure::getMindist_ZC(osp.surfacePoints[a], osp.surfacePoints[b], osp.surfacePoints[c], osp.surfacePoints[d], osp.cm);
					ZCRunTime+=tmr.elapsed();
					bool isFCZC = mindistZC > 0;

					double diff=mindistZC-mindistQhull;
					// check FC
					CHECK_EQUAL(isFCQhull, isFCZC);
					if (isFCQhull ^ isFCZC){
						printf("Qhull:%lf | ZC: %lf | diff %lf\n",mindistQhull,mindistZC,diff);
//						printf("%d %d %d %d\n",a,b,c,d);
						printf("%.9lf %.9lf %.9lf %.9lf %.9lf %.9lf\n",osp.surfacePoints[a].position.x(),osp.surfacePoints[a].position.y(),osp.surfacePoints[a].position.z(),osp.surfacePoints[a].normal.x(),osp.surfacePoints[a].normal.y(),osp.surfacePoints[a].normal.z());
						printf("%.9lf %.9lf %.9lf %.9lf %.9lf %.9lf\n",osp.surfacePoints[b].position.x(),osp.surfacePoints[b].position.y(),osp.surfacePoints[b].position.z(),osp.surfacePoints[b].normal.x(),osp.surfacePoints[b].normal.y(),osp.surfacePoints[b].normal.z());
						printf("%.9lf %.9lf %.9lf %.9lf %.9lf %.9lf\n",osp.surfacePoints[c].position.x(),osp.surfacePoints[c].position.y(),osp.surfacePoints[c].position.z(),osp.surfacePoints[c].normal.x(),osp.surfacePoints[c].normal.y(),osp.surfacePoints[c].normal.z());
						printf("%.9lf %.9lf %.9lf %.9lf %.9lf %.9lf\n",osp.surfacePoints[d].position.x(),osp.surfacePoints[d].position.y(),osp.surfacePoints[d].position.z(),osp.surfacePoints[d].normal.x(),osp.surfacePoints[d].normal.y(),osp.surfacePoints[d].normal.z());
						printf("mindistQhull %lf | mindistZC %lf\n",mindistQhull,mindistZC);
					}

					if(isFCZC){
						// check min dist
						CHECK(diff>0);
						CHECK(diff<2e-3);
						if(!(diff<2e-3)){
							printf("Qhull:%lf | ZC: %lf | diff %lf\n",mindistQhull,mindistZC,diff);
						}
					}
				}
			}
		}
	}
	printf("QHull: %lf\nZC: %lf\n",qHullRunTime,ZCRunTime);
}

}
