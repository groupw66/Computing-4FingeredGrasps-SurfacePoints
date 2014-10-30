#include "Compute4FingeredGrasps.h"

void Compute4FingeredGrasps::compute4FingeredGrasps(std::vector<std::vector<Grasp> > &sol, const std::vector<SurfacePoint> &surfacePoints, const std::vector<Eigen::Vector3d> &samplePoints, double halfAngle)
{
    sol.clear();
    sol.resize(samplePoints.size());
    #pragma omp parallel for schedule(static, 1)
    for(unsigned int i=0 ; i<samplePoints.size() ; ++i){
        std::vector<unsigned int> M;
        pointInConesFilter(M, surfacePoints, samplePoints[i], halfAngle);
        if(M.size() >= 4){
            std::vector<Grasp> fcGrasps;
            findEquilibriumGrasps_forceDual(fcGrasps, M, samplePoints[i], surfacePoints);
            sol[i] = fcGrasps;
        }
    }
}
std::vector<int> Compute4FingeredGrasps::compute4FingeredGrasps(std::set<Grasp>& sol, const std::vector<SurfacePoint>& surfacePoints, const std::vector<Eigen::Vector3d>& samplePoints, double halfAngle)
{
    std::vector<int> sizeSols;
    sol.clear();
    #pragma omp parallel for schedule(static, 1)
    for(unsigned int i=0 ; i<samplePoints.size() ; ++i){
        std::vector<unsigned int> M;
        pointInConesFilter(M, surfacePoints, samplePoints[i], halfAngle);
        std::vector<Grasp> fcGrasps;
        if(M.size() >= 4){
            //findEquilibriumGrasps_naive(fcGrasps, M, samplePoints[i]);
            findEquilibriumGrasps_forceDual(fcGrasps, M, samplePoints[i], surfacePoints);
        }
        #pragma omp critical
        {
            sol.insert(fcGrasps.begin(),fcGrasps.end());
            sizeSols.push_back(sol.size());
        }
    }
    return sizeSols;
}


void Compute4FingeredGrasps::compute4FingeredGrasps_naive(std::vector<std::vector<Grasp> > &sol, const std::vector<SurfacePoint> &surfacePoints, const std::vector<Eigen::Vector3d> &samplePoints, double halfAngle)
{
    sol.clear();
    sol.resize(samplePoints.size());
    #pragma omp parallel for schedule(static, 1)
    for(unsigned int i=0 ; i<samplePoints.size() ; ++i){
        std::vector<unsigned int> M;
        pointInConesFilter(M, surfacePoints, samplePoints[i], halfAngle);
        if(M.size() >= 4){
            std::vector<Grasp> fcGrasps;
            findEquilibriumGrasps_naive(fcGrasps, M, samplePoints[i], surfacePoints);
            sol[i] = fcGrasps;
        }
    }
}

void Compute4FingeredGrasps::pointInConesFilter(std::vector<unsigned int> &filtereds, const std::vector<SurfacePoint>& surfacePoints, Eigen::Vector3d point, double halfAngle)
{
    filtereds.clear();
    for(unsigned int i=0 ; i<surfacePoints.size() ; ++i){
        if(isPointInConeDoubleside(point, surfacePoints[i], halfAngle)){
            filtereds.push_back(i);
        }
	}
}

void Compute4FingeredGrasps::findEquilibriumGrasps_forceDual(std::vector<Grasp>& fcGrasps, const std::vector<unsigned int>& M, Eigen::Vector3d samplePoint, std::vector<SurfacePoint> surfacePoints)
{
    fcGrasps.clear();
	RangeTree2D PRN,
                PLP,
                PRP;
    std::vector<Eigen::Vector3d> vectorInwards;
    for(unsigned int surfacePointId : M){
        vectorInwards.push_back(findVectorInward(samplePoint, surfacePoints[surfacePointId].position, surfacePoints[surfacePointId].normal));
    }
	for(unsigned int i=0 ; i < vectorInwards.size() ; ++i){
		for(unsigned int j=i+1 ; j < vectorInwards.size() ; ++j){
			std::vector<int> VLNId,VLPId,VRNId,VRPId,VLZId,VRZId;
			std::vector<Eigen::Vector2d> VLP,VRN,VRP,fdrAngles;
			fdrAngles.reserve(vectorInwards.size()-j-1);
			// calculate T
			Eigen::Vector3d Tx = vectorInwards[i].cross(vectorInwards[j]).normalized(),
                            Tz = (vectorInwards[i] + vectorInwards[j]).normalized(),
                            Ty = Tz.cross(Tx);
            Eigen::Matrix3d T;
            T << Tx,Ty,Tz;
            T.transposeInPlace();
			double fanHalfLen=fabs(Geometry::toFDR(T*vectorInwards[i]).y());
			// partition points into appropriate V*
			for(unsigned int k=j+1 ; k < vectorInwards.size() ; ++k){
				Eigen::Vector3d n = T*vectorInwards[k];
				Eigen::Vector2d fdr = Geometry::toFDR(n);
				fdrAngles.push_back(Geometry::toFDRAngle(n, fanHalfLen));
				if(fdr.x() != 0){
					n.z() == 0 ?
						(fdr.x() < 0 ? VLZId : VRZId).push_back(k-j-1)
					:
						n.z()<0?
							(fdr.x() < 0 ? VLNId : VRNId).push_back(k-j-1)
						:
							(fdr.x() < 0 ? VLPId : VRPId).push_back(k-j-1);
				}
			}
			// make range tree
			sort(VRNId.begin(),VRNId.end(), [&](int a,int b){return fdrAngles[a].x() == fdrAngles[b].x() ? fdrAngles[a].y() < fdrAngles[b].y() : fdrAngles[a].x() < fdrAngles[b].x() ;});
			sort(VLPId.begin(),VLPId.end(), [&](int a,int b){return fdrAngles[a].x() == fdrAngles[b].x() ? fdrAngles[a].y() < fdrAngles[b].y() : fdrAngles[a].x() < fdrAngles[b].x() ;});
			sort(VRPId.begin(),VRPId.end(), [&](int a,int b){return fdrAngles[a].x() == fdrAngles[b].x() ? fdrAngles[a].y() < fdrAngles[b].y() : fdrAngles[a].x() < fdrAngles[b].x() ;});

			VRN.reserve(VRNId.size());
			for(int id : VRNId)
				VRN.push_back(fdrAngles[id]);
			VLP.reserve(VLPId.size());
			for(int id : VLPId)
				VLP.push_back(fdrAngles[id]);
			VRP.reserve(VRPId.size());
			for(int id : VRPId)
				VRP.push_back(fdrAngles[id]);
			PRN.init(VRN);
			PLP.init(VLP);
			PRP.init(VRP);
			int lnrn = 0,
                lnlp = 0,
                rnrp = 0;
			for(int id : VLNId){
				// Negative-Negative Dual Points Pairing
				for(int l : PRN.queryId(0,0, M_PI-fdrAngles[id].x(), M_PI-fdrAngles[id].y())){
                    fcGrasps.push_back(Grasp(M[i], M[j], M[j+1+id], M[j+1+VRNId[l]]));
					++lnrn;
				}
				// Negative-Positive Dual Points Pairing
				// Left_Negative / Left_Positive
				for(int l : PLP.queryId(fdrAngles[id].x(), fdrAngles[id].y(), M_PI,M_PI)){
                    fcGrasps.push_back(Grasp(M[i], M[j], M[j+1+id], M[j+1+VLPId[l]]));
					++lnlp;
				}
			}

			// Negative-Zero Dual Points Pairing
			// TODO

			// Negative-Positive Dual Points Pairing
			// Right_Negative / Right_Positive
			for(int id : VRNId){
				for(int l : PRP.queryId(fdrAngles[id].x(), fdrAngles[id].y(), M_PI,M_PI)){
					fcGrasps.push_back(Grasp(M[i], M[j], M[j+1+id], M[j+1+VRPId[l]]));
					++rnrp;
				}
			}
			//printf("%d %d %d\n",lnrn,lnlp,rnrp);

		}
	}
}

void Compute4FingeredGrasps::findEquilibriumGrasps_forceDual(std::vector<std::tuple<double, Grasp, double> >& sols, std::unordered_set<std::string>& solsSet, Timer& tmr, double timelimit, double halfAngle, const std::vector<unsigned int>& M, Eigen::Vector3d samplePoint, std::vector<SurfacePoint> surfacePoints)
{
    //fcGrasps.clear();
	RangeTree2D PRN,
                PLP,
                PRP;
    std::vector<Eigen::Vector3d> vectorInwards;
    for(unsigned int surfacePointId : M){
        vectorInwards.push_back(findVectorInward(samplePoint, surfacePoints[surfacePointId].position, surfacePoints[surfacePointId].normal));
    }
	for(unsigned int i=0 ; i < vectorInwards.size() ; ++i){
		for(unsigned int j=i+1 ; j < vectorInwards.size() ; ++j){
            if(tmr.elapsed() >= timelimit)
                break;
			std::vector<int> VLNId,VLPId,VRNId,VRPId,VLZId,VRZId;
			std::vector<Eigen::Vector2d> VLP,VRN,VRP,fdrAngles;
			fdrAngles.reserve(vectorInwards.size()-j-1);
			// calculate T
			Eigen::Vector3d Tx = vectorInwards[i].cross(vectorInwards[j]).normalized(),
                            Tz = (vectorInwards[i] + vectorInwards[j]).normalized(),
                            Ty = Tz.cross(Tx);
            Eigen::Matrix3d T;
            T << Tx,Ty,Tz;
            T.transposeInPlace();
			double fanHalfLen=fabs(Geometry::toFDR(T*vectorInwards[i]).y());
			// partition points into appropriate V*
			for(unsigned int k=j+1 ; k < vectorInwards.size() ; ++k){
				Eigen::Vector3d n = T*vectorInwards[k];
				Eigen::Vector2d fdr = Geometry::toFDR(n);
				fdrAngles.push_back(Geometry::toFDRAngle(n, fanHalfLen));
				if(fdr.x() != 0){
					n.z() == 0 ?
						(fdr.x() < 0 ? VLZId : VRZId).push_back(k-j-1)
					:
						n.z()<0?
							(fdr.x() < 0 ? VLNId : VRNId).push_back(k-j-1)
						:
							(fdr.x() < 0 ? VLPId : VRPId).push_back(k-j-1);
				}
			}
			// make range tree
			sort(VRNId.begin(),VRNId.end(), [&](int a,int b){return fdrAngles[a].x() == fdrAngles[b].x() ? fdrAngles[a].y() < fdrAngles[b].y() : fdrAngles[a].x() < fdrAngles[b].x() ;});
			sort(VLPId.begin(),VLPId.end(), [&](int a,int b){return fdrAngles[a].x() == fdrAngles[b].x() ? fdrAngles[a].y() < fdrAngles[b].y() : fdrAngles[a].x() < fdrAngles[b].x() ;});
			sort(VRPId.begin(),VRPId.end(), [&](int a,int b){return fdrAngles[a].x() == fdrAngles[b].x() ? fdrAngles[a].y() < fdrAngles[b].y() : fdrAngles[a].x() < fdrAngles[b].x() ;});

			VRN.reserve(VRNId.size());
			for(int id : VRNId)
				VRN.push_back(fdrAngles[id]);
			VLP.reserve(VLPId.size());
			for(int id : VLPId)
				VLP.push_back(fdrAngles[id]);
			VRP.reserve(VRPId.size());
			for(int id : VRPId)
				VRP.push_back(fdrAngles[id]);
			PRN.init(VRN);
			PLP.init(VLP);
			PRP.init(VRP);
			int lnrn = 0,
                lnlp = 0,
                rnrp = 0;
			for(int id : VLNId){
				// Negative-Negative Dual Points Pairing
				for(int l : PRN.queryId(0,0, M_PI-fdrAngles[id].x(), M_PI-fdrAngles[id].y())){
                    Grasp g(M[i], M[j], M[j+1+id], M[j+1+VRNId[l]]);
                    if(!solsSet.insert(g.to_str()).second)
                        continue;
                    double mindist = ForceClosure::getMindist_ZC(surfacePoints[g[0]], surfacePoints[g[1]],
                                                             surfacePoints[g[2]], surfacePoints[g[3]],
                                                             Eigen::Vector3d(0,0,0), halfAngle);
                    if(tmr.elapsed() >= timelimit)
                        break;
                    sols.push_back(std::make_tuple(mindist, g, tmr.elapsed()));
                    //sols.push_back(Grasp(M[i], M[j], M[j+1+id], M[j+1+VRNId[l]]));
					++lnrn;
				}
				// Negative-Positive Dual Points Pairing
				// Left_Negative / Left_Positive
				for(int l : PLP.queryId(fdrAngles[id].x(), fdrAngles[id].y(), M_PI,M_PI)){
                    Grasp g(M[i], M[j], M[j+1+id], M[j+1+VLPId[l]]);
                    if(!solsSet.insert(g.to_str()).second)
                        continue;
                    double mindist = ForceClosure::getMindist_ZC(surfacePoints[g[0]], surfacePoints[g[1]],
                                                             surfacePoints[g[2]], surfacePoints[g[3]],
                                                             Eigen::Vector3d(0,0,0), halfAngle);
                    if(tmr.elapsed() >= timelimit)
                        break;
                    sols.push_back(std::make_tuple(mindist, g, tmr.elapsed()));
                    //sols.push_back(Grasp(M[i], M[j], M[j+1+id], M[j+1+VLPId[l]]));
					++lnlp;
				}
			}

			// Negative-Zero Dual Points Pairing
			// TODO

			// Negative-Positive Dual Points Pairing
			// Right_Negative / Right_Positive
			for(int id : VRNId){
				for(int l : PRP.queryId(fdrAngles[id].x(), fdrAngles[id].y(), M_PI,M_PI)){
					Grasp g(M[i], M[j], M[j+1+id], M[j+1+VRPId[l]]);
                    if(!solsSet.insert(g.to_str()).second)
                        continue;
                    double mindist = ForceClosure::getMindist_ZC(surfacePoints[g[0]], surfacePoints[g[1]],
                                                             surfacePoints[g[2]], surfacePoints[g[3]],
                                                             Eigen::Vector3d(0,0,0), halfAngle);
                    if(tmr.elapsed() >= timelimit)
                        break;
                    sols.push_back(std::make_tuple(mindist, g, tmr.elapsed()));
                    //sols.push_back(Grasp(M[i], M[j], M[j+1+id], M[j+1+VRPId[l]]));
					++rnrp;
				}
			}
			//printf("%d %d %d\n",lnrn,lnlp,rnrp);

		}
	}
}


void Compute4FingeredGrasps::findEquilibriumGrasps_naive(std::vector<Grasp>  &sol, const std::vector<unsigned int>& M, Eigen::Vector3d samplePoint, std::vector<SurfacePoint> surfacePoints)
{
    sol.clear();
    for(unsigned int a=0 ; a < M.size() ; ++a){
        for(unsigned int b=a+1 ; b < M.size() ; ++b){
            for(unsigned int c=b+1 ; c < M.size() ; ++c){
                for(unsigned int d=c+1 ; d < M.size() ; ++d){
                    Grasp grasp(M[a], M[b], M[c], M[d]);
                    if(isEquilibriumGrasp(grasp, samplePoint, surfacePoints)){
                       sol.push_back(grasp);
                    }
                }
            }
        }
    }
}

bool Compute4FingeredGrasps::isEquilibriumGrasp(Grasp grasp, Eigen::Vector3d samplePoint, std::vector<SurfacePoint> surfacePoints)
{
    std::vector<Eigen::Vector3d> vectorInwards;
    for(int surfacePointId : grasp.surfacePoints){
        vectorInwards.push_back( findVectorInward(samplePoint, surfacePoints[surfacePointId].position, surfacePoints[surfacePointId].normal));
    }
    return Geometry::isVectorsPositivelySpan3D(vectorInwards);
}

