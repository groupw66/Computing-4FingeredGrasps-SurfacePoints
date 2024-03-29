#include <UnitTest++.h>
#include <iostream>
#include <Eigen/Dense>
#include "ForceClosure.h"
#include "PositionsNormalsFile.h"
#include "ObjectSurfacePoints.h"
#include "Timer.h"
#include "dae/DaeHeuristic.h"
namespace
{

TEST(ForceClosure_getMindist_Qhull)
{
    //TO-DO
}

TEST(ForceClosure_near_Qhull_ZC)
{
	PositionsNormalsFile obj("test/meshes/spectralMesh/cow500.txt");
	ObjectSurfacePoints osp(obj);
	unsigned int nSurfacePoint = osp.surfacePoints.size();
	int step=1<<7;
	Timer tmr;
	double qHullRunTime=0.,ZCRunTime=0.;
	for(unsigned int a=0 ; a<nSurfacePoint ; a+=step){
		for(unsigned int b=a+1 ; b<nSurfacePoint ; b+=step){
//			printf("SurfacePointId: %d\n",b);
			for(unsigned int c=b+1 ; c<nSurfacePoint ; c+=step){
				for(unsigned int d=c+1 ; d<nSurfacePoint ; d+=step){
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
					if(isFCQhull){
						CHECK(isFCZC || fabs(diff)<1e-7);
					}
					else{
						CHECK(!isFCZC || diff<2e-4);
					}
					if (isFCQhull ^ isFCZC){
						printf("%d %d %d %d\n",a,b,c,d);
						printf("Qhull:%lf | ZC: %lf | diff %lf\n",mindistQhull,mindistZC,diff);
						std::cout<<osp.surfacePoints[a].position.transpose()<<" ";
						std::cout<<osp.surfacePoints[a].normal.transpose()<<std::endl;
						std::cout<<osp.surfacePoints[b].position.transpose()<<" ";
						std::cout<<osp.surfacePoints[b].normal.transpose()<<std::endl;
						std::cout<<osp.surfacePoints[c].position.transpose()<<" ";
						std::cout<<osp.surfacePoints[c].normal.transpose()<<std::endl;
						std::cout<<osp.surfacePoints[d].position.transpose()<<" ";
						std::cout<<osp.surfacePoints[d].normal.transpose()<<std::endl;
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
//*
TEST(Random_ForceClosure)
{
	PositionsNormalsFile obj("test/meshes/spectralMesh/cow500.txt");
	ObjectSurfacePoints osp(obj);
	unsigned int nSurfacePoint = osp.surfacePoints.size();
	int nSol=0;
	std::uniform_int_distribution<> rand(0,nSurfacePoint-1);
	std::default_random_engine rng;
//	rng.seed(std::random_device());
	DaeHeuristicChecker daeHeuristicChecker(M_PI / 18.);
	Timer tmr;
//	while(nSol<107820){
	while(nSol<6444){
		int a=rand(rng),b=rand(rng),c=rand(rng),d=rand(rng);
//		if(ForceClosure::isFC_ZC(osp.surfacePoints[a],osp.surfacePoints[b],osp.surfacePoints[c],osp.surfacePoints[d],osp.cm))
		if(daeHeuristicChecker.isForceClosure(osp.surfacePoints[a], osp.surfacePoints[b], osp.surfacePoints[c], osp.surfacePoints[d]) && (ForceClosure::isFC_ZC(osp.surfacePoints[a],osp.surfacePoints[b],osp.surfacePoints[c],osp.surfacePoints[d],osp.cm)))
		{
			++nSol;
//			printf("%d %lf\n",nSol,tmr.elapsed());
		}
	}
	printf("Found %d soltions in %lf second\n",nSol,tmr.elapsed());
}
//*/
}
