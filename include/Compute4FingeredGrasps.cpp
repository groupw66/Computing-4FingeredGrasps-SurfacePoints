#include "Compute4FingeredGrasps.h"

void Compute4FingeredGrasps::compute4FingeredGrasps(std::vector<std::vector<Grasp> > &sol, const std::vector<SurfacePoint> &surfacePoints, const std::vector<Eigen::Vector3d> &samplePoints, double halfAngle)
{
    sol.clear();
    sol.resize(samplePoints.size());
    #pragma omp parallel for schedule(static, 1)
    for(int i=0 ; i<samplePoints.size() ; ++i){
        std::vector<SurfacePoint> M;
        pointInConesFilter(M, surfacePoints, samplePoints[i], halfAngle);
        if(M.size() >= 4){
            std::vector<Grasp> fcGrasps;
            findEquilibriumGrasps_naive(fcGrasps, M, samplePoints[i]);
            sol[i] = fcGrasps;
        }
    }
}

void Compute4FingeredGrasps::pointInConesFilter(std::vector<SurfacePoint> &filtereds, const std::vector<SurfacePoint>& surfacePoints, Eigen::Vector3d point, double halfAngle)
{
    filtereds.clear();
    for(const SurfacePoint &surfacePoint : surfacePoints){
        if(isPointInConeDoubleside(point, surfacePoint, halfAngle)){
            filtereds.push_back(surfacePoint);
        }
	}
}

void Compute4FingeredGrasps::findEquilibriumGrasps_naive(std::vector<Grasp>  &sol, const std::vector<SurfacePoint>& M, Eigen::Vector3d samplePoint)
{
    sol.clear();
    for(int a=0 ; a < M.size() ; ++a){
        for(int b=a+1 ; b < M.size() ; ++b){
            for(int c=b+1 ; c < M.size() ; ++c){
                for(int d=c+1 ; d < M.size() ; ++d){
                    SurfacePoint aa = M[a];
                    SurfacePoint bb = M[b];
                    SurfacePoint cc = M[c];
                    SurfacePoint dd = M[d];
                    Grasp grasp(aa,bb,cc,dd);
                    if(isEquilibriumGrasp(grasp,samplePoint)){
                       sol.push_back(grasp);
                    }
                }
            }
        }
    }
}

bool Compute4FingeredGrasps::isEquilibriumGrasp(Grasp grasp, Eigen::Vector3d point)
{
    std::vector<Eigen::Vector3d> vectorInwards;
    for(SurfacePoint surfacePoint : grasp.surfacePoints){
        vectorInwards.push_back( findVectorInward(point, surfacePoint.position, surfacePoint.normal) );
    }
    return Geometry::isVectorsPositivelySpan3D(vectorInwards);
}

