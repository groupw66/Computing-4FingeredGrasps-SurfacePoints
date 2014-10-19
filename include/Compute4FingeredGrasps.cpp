#include "Compute4FingeredGrasps.h"

std::vector<std::vector<Grasp> > Compute4FingeredGrasps::Compute4FingeredGrasps(const std::vector<SurfacePoint> &surfacePoints, const std::vector<Eigen::Vector3d> &samplePoints, double halfAngle)
{
    std::vector<std::vector<Grasp> > sol;
    #pragma omp parallel for schedule(static, 1)
    for(int i=0 ; i<samplePoints.size() ; ++i){
        std::vector<SurfacePoint> M = pointInConesFilter(surfacePoints, samplePoints[i], halfAngle);
        if(M.size() >= 4){
            std::vector<Grasp> fcGrasps = findEquilibriumGrasps_naive(M, samplePoints[i]);
            sol.push_back(fcGrasps);
        }
    }
    return sol;
}

std::vector<SurfacePoint> Compute4FingeredGrasps::pointInConesFilter(const std::vector<SurfacePoint>& surfacePoints, Eigen::Vector3d point, double halfAngle)
{
    std::vector<SurfacePoint> filtereds;
    for(const SurfacePoint &surfacePoint : surfacePoints){
        if(isPointInConeDoubleside(point, surfacePoint, halfAngle)){
            filtereds.push_back(surfacePoint);
        }
	}
	return filtereds;
}

std::vector<Grasp> Compute4FingeredGrasps::findEquilibriumGrasps_naive(const std::vector<SurfacePoint>& M, Eigen::Vector3d samplePoint)
{
    std::vector<Grasp> sol;
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
    return sol;
}

bool Compute4FingeredGrasps::isEquilibriumGrasp(Grasp grasp, Eigen::Vector3d point)
{
    /*
    Eigen::Vector3d a = findVectorInward(_point, contactNormals[_grasp(0)], contactPositions[_grasp(0)]);
    Eigen::Vector3d b = findVectorInward(_point, contactNormals[_grasp(1)], contactPositions[_grasp(1)]);
    Eigen::Vector3d c = findVectorInward(_point, contactNormals[_grasp(2)], contactPositions[_grasp(2)]);
    Eigen::Vector3d d = findVectorInward(_point, contactNormals[_grasp(3)], contactPositions[_grasp(3)]);
    return isVectorsPositivelySpan3D(a, b, c, d);
    */
    return true;
}

