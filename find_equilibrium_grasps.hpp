#ifndef FIND_EQUILIBRIUM_GRASPS_HPP_INCLUDED
#define FIND_EQUILIBRIUM_GRASPS_HPP_INCLUDED

#include<vector>
#include<algorithm>
#include"printDebug.hpp"
using namespace std;

inline bool isPositiveSpan(const Point3d &n1,const Point3d &n2,const Point3d &n3,const Point3d &n4){
	// ported from isFC2d fc.cpp
	orgQhull::Qhull qhull;
	realT points[12]={n1.x(),n1.y(),n1.z(),n2.x(),n2.y(),n2.z(),n3.x(),n3.y(),n3.z(),n4.x(),n4.y(),n4.z()};
//printf("%s%s%s%s\n",print(n1),print(n2),print(n3),print(n4));
	qhull.runQhull("",3,4,points,"");
//printf("%d %d\n",qhull.points().count(),qhull.facetList().size());
	for(const orgQhull::QhullFacet &facet : qhull.facetList().toStdVector())
		if(facet.getFacetT()->offset>=0) return false;
	return true;
}

inline vector<Grasp> find_equilibrium_grasps_naive(const vector<pair<PointNormal,Point3d> > &M){
	// test every combination
	// O(n^4)
	vector<Grasp> fcGrasps;
	for(vector<pair<PointNormal,Point3d> >::const_iterator i=M.begin();i!=M.end();++i){
		for(vector<pair<PointNormal,Point3d> >::const_iterator j=i+1;j!=M.end();++j){
			for(vector<pair<PointNormal,Point3d> >::const_iterator k=j+1;k!=M.end();++k){
				for(vector<pair<PointNormal,Point3d> >::const_iterator l=k+1;l!=M.end();++l){
					if(isPositiveSpan(i->second,j->second,k->second,l->second)){
						fcGrasps.push_back(Grasp(i->first,j->first,k->first,l->first));
					}
				}
			}
		}
	}
	return fcGrasps;
}

inline Point2d floor(const Point2d &a,const Point2d &b){
	return Point2d(min(a.first,b.first),min(a.second,b.second));
}

inline Point2d ceil(const Point2d &a,const Point2d &b){
	return Point2d(max(a.first,b.first),max(a.second,b.second));
}

inline vector<Grasp> find_equilibrium_grasps_sph(vector<pair<PointNormal,Point3d> > &M){
	// using orthogonal range search with fractional cascading in spherical coordinate
	// O(n^3 (logn + K))
	// TODO deal with southpole and 0,2pi case

	vector<Point2d> sph,sphN;
	sph.reserve(M.size());
	sphN.reserve(M.size());

	// sort M by sph
	sort(M.begin(),M.end(),[](const pair<PointNormal,Point3d> &a,const pair<PointNormal,Point3d> &b){return a.second.toSph()<b.second.toSph();});
	for(const pair<PointNormal,Point3d> &pn : M){
		sph.push_back(pn.second.toSph());
		sphN.push_back((-pn.second).toSph());
	}

	// make range tree with spherical coordinate
	RangeTree rt(sph);
	vector<Grasp> fcGrasps;
	for(vector<pair<PointNormal,Point3d> >::const_iterator i=M.begin();i!=M.end();++i){
		for(vector<pair<PointNormal,Point3d> >::const_iterator j=i+1;j!=M.end();++j){
			Point2d minj=floor(sphN[i-M.begin()],sphN[j-M.begin()]),maxj=ceil(sphN[i-M.begin()],sphN[j-M.begin()]);
			for(vector<pair<PointNormal,Point3d> >::const_iterator k=j+1;k!=M.end();++k){
				Point2d mink=floor(minj,sphN[k-M.begin()]),maxk=ceil(maxj,sphN[k-M.begin()]);
				// search for point in box mink - maxk
				vector<int> pointId=rt.queryId(mink.first,mink.second,maxk.first,maxk.second);
//				for(vector<int>::iterator l=upper_bound(pointId.begin(),pointId.end(),k);
				for(vector<int>::iterator l=pointId.begin();
					l!=pointId.end();
					++l){
//printf("%d %d\n",*l,pointId.end()-l);
					if(*l>k-M.begin() && isPositiveSpan(i->second,j->second,k->second,M[*l].second)){
//if(M[i].inverse||M[j].inverse||M[k].inverse||M[*l].inverse) printf("FC using another side of cone\n");
						fcGrasps.push_back(Grasp(i->first,j->first,k->first,M[*l].first));
					}
				}
			}
		}
	}
	return fcGrasps;
}

inline vector<Grasp> find_equilibrium_grasps(const vector<pair<PointNormal,Point3d> > &M){
	// using orthogonal range search with fractional cascading in force dual representation
	// O(n^3 (logn + K))

	vector<Grasp> fcGrasps;
	RangeTree PRN,
			PLP,
			PRP;
	for(vector<pair<PointNormal,Point3d> >::const_iterator i=M.begin();i!=M.end();++i){
//printf("%d\n",i);
		for(vector<pair<PointNormal,Point3d> >::const_iterator j=i+1;j!=M.end();++j){
			vector<int> VLNId,VLPId,VRNId,VRPId,VLZId,VRZId;
			vector<Point2d> VLP,VRN,VRP,fdrAngles;
			fdrAngles.reserve(M.end()-j-1);
			// calculate T
			Point3d Tx=i->second.cross(j->second).normalized(),
					Tz=(i->second+j->second).normalized(),
					Ty=Tz.cross(Tx);
//			double fanHalfLen=(M[i].n.rotate(Tx,Ty,Tz).toFDR() - M[j].n.rotate(Tx,Ty,Tz).toFDR()).norm2()/2;
			double fanHalfLen=fabs(i->second.rotate(Tx,Ty,Tz).toFDR().second);
//printf("%s | %s",print(M[i].n,false),print(M[j].n));
//printf("\t%s | %s",print(M[i].n.rotate(Tx,Ty,Tz),false),print(M[j].n.rotate(Tx,Ty,Tz)));
//auto wa=M[i].n.rotate(Tx,Ty,Tz).toFDR(),wb=M[j].n.rotate(Tx,Ty,Tz).toFDR();
//printf("\t\t%lf %lf | %lf %lf | Len=%lf\n",wa.first,wa.second,wb.first,wb.second,fanHalfLen);
			// partition points into appropriate V*
			for(vector<pair<PointNormal,Point3d> >::const_iterator k=j+1;k!=M.end();++k){
				Point3d n=k->second.rotate(Tx,Ty,Tz);
				Point2d fdr=n.toFDR();
				fdrAngles.push_back(n.toFDRAngle(fanHalfLen));
				if(fdr.first!=0){
					n.z()==0?
						(fdr.first<0?VLZId:VRZId).push_back(k-j-1)
					:
						n.z()<0?
							(fdr.first<0?VLNId:VRNId).push_back(k-j-1)
						:
							(fdr.first<0?VLPId:VRPId).push_back(k-j-1);
				}
			}

//printf("%d | %d %d %d %d\n",fdrAngles.size(),VLNId.size(),VLPId.size(),VRNId.size(),VRPId.size());
			// make range tree
			sort(VRNId.begin(),VRNId.end(), [&](int a,int b){return fdrAngles[a]<fdrAngles[b];});
			sort(VLPId.begin(),VLPId.end(), [&](int a,int b){return fdrAngles[a]<fdrAngles[b];});
			sort(VRPId.begin(),VRPId.end(), [&](int a,int b){return fdrAngles[a]<fdrAngles[b];});

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
			for(int id : VLNId){
				// Negative-Negative Dual Points Pairing
				for(int l : PRN.queryId(0,0, M_PI-fdrAngles[id].first, M_PI-fdrAngles[id].second))
					fcGrasps.push_back(Grasp(i->first, j->first, (j+1+id)->first, (j+1+VRNId[l])->first));
				// Negative-Positive Dual Points Pairing
				// Left_Negative / Left_Positive
				for(int l : PLP.queryId(fdrAngles[id].first, fdrAngles[id].second, M_PI,M_PI))
					fcGrasps.push_back(Grasp(i->first, j->first, (j+1+id)->first, (j+1+VLPId[l])->first));
			}

			// Negative-Zero Dual Points Pairing
			// TODO

			// Negative-Positive Dual Points Pairing
			// Right_Negative / Right_Positive
			for(int id : VRNId){
				for(int l : PRP.queryId(fdrAngles[id].first, fdrAngles[id].second, M_PI,M_PI))
					fcGrasps.push_back(Grasp(i->first, j->first, (j+1+id)->first, (j+1+VRPId[l])->first));
			}

		}
	}
	return fcGrasps;
}

#endif // FIND_EQUILIBRIUM_GRASPS_HPP_INCLUDED
