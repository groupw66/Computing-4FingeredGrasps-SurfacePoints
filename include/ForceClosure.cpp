#include "ForceClosure.h"

double ForceClosure::getMindist_Qhull(SurfacePoint sp1, SurfacePoint sp2, SurfacePoint sp3, SurfacePoint sp4, Eigen::Vector3d cm, double halfAngle, int nPyramidSide)
{
    std::vector<Wrench> wrenchs;
    std::vector<Wrench> tmpWrenchs;
    tmpWrenchs = sp1.getWrenchCone(cm, halfAngle, nPyramidSide);
    wrenchs.insert(wrenchs.end(), tmpWrenchs.begin(), tmpWrenchs.end());
    tmpWrenchs = sp2.getWrenchCone(cm, halfAngle, nPyramidSide);
    wrenchs.insert(wrenchs.end(), tmpWrenchs.begin(), tmpWrenchs.end());
    tmpWrenchs = sp3.getWrenchCone(cm, halfAngle, nPyramidSide);
    wrenchs.insert(wrenchs.end(), tmpWrenchs.begin(), tmpWrenchs.end());
    tmpWrenchs = sp4.getWrenchCone(cm, halfAngle, nPyramidSide);
    wrenchs.insert(wrenchs.end(), tmpWrenchs.begin(), tmpWrenchs.end());
    return getMindist_Qhull(wrenchs);
}

double ForceClosure::getMindist_Qhull(std::vector<Wrench> wrenchs)
{
    double mindist = std::numeric_limits<double>::max();
    orgQhull::Qhull qhull;
    realT points[6 * wrenchs.size()];
    for(unsigned int i=0 ; i < wrenchs.size() ; ++i){
        points[i*6] = wrenchs[i](0);
        points[i*6+1] = wrenchs[i](1);
        points[i*6+2] = wrenchs[i](2);
        points[i*6+3] = wrenchs[i](3);
        points[i*6+4] = wrenchs[i](4);
        points[i*6+5] = wrenchs[i](5);
    }
    qhull.runQhull("", 6, wrenchs.size(), points, "");
    std::vector<orgQhull::QhullFacet> facets=qhull.facetList().toStdVector();
    for(const orgQhull::QhullFacet &facet : qhull.facetList().toStdVector()){
        mindist = std::min(mindist, -facet.getFacetT()->offset);
    }
    return mindist;
}


inline std::pair<double, Eigen::Matrix<double,6,1> > SuppFunL1(const Eigen::Matrix<double,6,1> &u, const std::vector<SurfacePoint> &Gi, double uFriction)
{
	Eigen::Matrix<double,6,1> Spoint;
	double Svalue = std::numeric_limits<double>::min();
	for (const SurfacePoint &G : Gi) {
		Eigen::Vector3d tran=u.topRows<3>() + u.bottomRows<3>().cross(G.position);
		double c=tran.dot(G.normal), cand=c + uFriction*G.normal.cross(tran).norm();
		if (Svalue<cand) {
			Svalue=cand;
			Spoint=Wrench(G.position, uFriction*((tran-c*G.normal).normalized()) + G.normal);
		}
	}
	return std::pair<double, Eigen::Matrix<double,6,1> >(Svalue, Spoint);
}

inline std::pair<double, Eigen::Matrix<double,6,1> > SuppFunLinf(const Eigen::Matrix<double,6,1> &u, const std::vector<SurfacePoint> &Gi, double uFriction)
{
	Eigen::Matrix<double,6,1> Spoint=Eigen::Matrix<double,6,1>::Zero();
	double Svalue = 0;
	for (const SurfacePoint &G : Gi) {
		Eigen::Vector3d tran=u.topRows<3>() + u.bottomRows<3>().cross(G.position);
		double c=tran.dot(G.normal), cand=c + uFriction*G.normal.cross(tran).norm();
		if (cand>0) {
			Svalue+=cand;
			Spoint+=Wrench(G.position, uFriction*((tran-c*G.normal).normalized()) + G.normal);
		}
	}
	return std::pair<double, Eigen::Matrix<double,6,1> >(Svalue, Spoint);
}


inline std::pair<double, Eigen::Matrix<double,6,1> > SuppFun(const Eigen::Matrix<double,6,1> &u, const std::vector<SurfacePoint> &Gi, double uFriction)
{
	return SuppFunL1(u, Gi, uFriction);
}

inline std::pair<Eigen::Matrix<double,6,1>, Eigen::MatrixXd> ZCSubAlgorithm(const Eigen::Matrix<double,6,1> &b, Eigen::MatrixXd A, Eigen::VectorXd c, double Tol)
{
	//	This subalgorithm is to find the minimal subset of 'A' such that the
	//	point on the convex cone of 'A' closest to 'b' can be written as a
	//	strictly positive combination 'subA'
	//	Note: the columns of 'A' are linearly independent, so that the convex
	//	cone of 'A' is a simplicial cone

	//	'r' --- vector from the closest point to the given point 'b'

	if ((b.transpose()*A).maxCoeff() < 0) {
		return std::pair<Eigen::Matrix<double,6,1>, Eigen::MatrixXd>(b, Eigen::MatrixXd(0,0));
	}

	//	If the coefficient vector 'c' has only one negative component, then the
	//	corresponding element in A must not belong to the desired minimal subset and can be removed.
	int num_neg = 0;
	int ind_neg;
	int l=A.cols();
	for (int i=0; i<l-1; ++i) {
		if (c(i)<0) {
			++num_neg;
			ind_neg=i;
		}
	}
	while (num_neg==1) {
		--l;
		// remove column #ind_neg
		A.block(0, ind_neg, 6, l-ind_neg) = A.rightCols(l-ind_neg);
		A.conservativeResize(Eigen::NoChange, l);
//		c = (A.transpose()*A).inverse()*A.transpose()*b;
		c = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
		num_neg = 0;
		for (int i=0; i<l-1; ++i) {
			if (c(i)<0) {
				++num_neg;
				ind_neg=i;
			}
		}
	}
	//	If the coefficient vector 'c' has more than one negative components,
	//	then all the subsets of 'A' are checked one by one
	Eigen::Matrix<double,6,1> r;
	Eigen::MatrixXd subA;

	if (num_neg > 1) {
		bool not_found=true;
		// initialize subA
		subA.resize(6, l-1);
		subA.col(0)=A.col(l-1);
		int num_pos=l-1-num_neg;
//		Eigen::MatrixXd ANeg(6,num_neg);
		std::vector<Eigen::Matrix<double,6,1> > ANeg,APos;
		ANeg.reserve(num_neg);
		APos.reserve(num_pos);
		for(int i=0;i<l-1;++i){
			(c(i)<0)?
				ANeg.push_back(A.col(i))
			:
				APos.push_back(A.col(i));
		}
		int l_=1; // number of removed element
		std::vector<bool> negIndices(num_neg), posIndices(num_pos);
		do {
			subA.conservativeResize(6, l-l_);
			int nNeg=std::min(l_,num_neg), nPos=l_-nNeg;
			std::fill_n(negIndices.begin(),nNeg,false);
			std::fill(negIndices.begin()+nNeg,negIndices.end(),true);
			std::fill_n(posIndices.begin(),nPos,false);
			std::fill(posIndices.begin()+nPos,posIndices.end(),true);
			do{
				do{
					do{
						// calculate next subA
						int iSubA=1;
						for(int i=0;i<num_neg;++i){
							if (negIndices[i]) {
								subA.col(iSubA++)=ANeg[i];
							}
						}
						for(int i=0;i<num_pos;++i){
							if (posIndices[i]) {
								subA.col(iSubA++)=APos[i];
							}
						}
		//				c = (subA.transpose()*subA).inverse()*subA.transpose()*b;
						c = subA.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
						if (c.minCoeff() > 0) {
							r = b - subA*c;
							if ((r.transpose()*A).maxCoeff() < Tol) {
								not_found = false;
								break;
							}
						}
					} while(std::next_permutation(posIndices.begin(), posIndices.end()));
				} while(not_found && std::next_permutation(negIndices.begin(), negIndices.end()));

				negIndices[--nNeg]=true;
				posIndices[nPos++]=false;
			} while(not_found && nNeg>0 && nPos<=num_pos);
			++l_;
		} while (not_found);
	}
	else{
		subA = A;
		r = b - A*c;
	}

	//	If any component of 'c' is zero, then the corresponding element in
	//	'subA' can be removed.
	for (int i=0; i<subA.cols(); ) {
		if (c(i)<Tol) { // rarely happened
			// remove column #i
			subA.block(0, i, 6, subA.cols()-i-1) = subA.rightCols(subA.cols()-i-1);
			subA.conservativeResize(Eigen::NoChange, subA.cols()-1);
		}
		else
			++i;
	}
	return std::pair<Eigen::Matrix<double,6,1>, Eigen::MatrixXd>(r, subA);
}

inline std::tuple<double, Eigen::Matrix<double,6,1>, Eigen::MatrixXd> ZCAlgorithm(const Eigen::Matrix<double,6,1> &b, const std::vector<SurfacePoint> &Gi, double uFriction, double Tol)
{
	// This algorithm computes the minimum distance between a convex cone and a point 'b' (IEEE T-RO'09)
	//	'd' --- the minimum distance
	//	'r' --- residual vector from the closest point to 'b'
	//	'A' --- a subset of linearly independent points in the convex cone such that 'b' can be written as their positive combination
	//	'l' --- number of columns of 'A'
	Eigen::Matrix<double,6,1> r = b;
	double d = r.norm();
	Eigen::MatrixXd A = Eigen::Matrix<double,6,0>();
	int l = 0;
	double Svalue;
	Eigen::Matrix<double,6,1> Spoint;
	std::tie(Svalue, Spoint) = SuppFun(r, Gi, uFriction);
	while (Svalue > Tol){
		A.conservativeResize(Eigen::NoChange, ++l);
		A.rightCols<1>()=Spoint;
//		Eigen::VectorXd c = (A.transpose()*A).inverse()*A.transpose()*b;
		Eigen::VectorXd c = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
//		printf("minc: %lf\n",c.minCoeff());
		if (c.minCoeff() < 0){
			std::tie(r, A) = ZCSubAlgorithm(b, A, c, Tol);
//			std::cout<<r;
			if(r.norm()>=d) break;
			d = r.norm();
			l = A.cols();
//			printf("%lf %lf %lf %lf %lf %lf | %lf\n",r(0),r(1),r(2),r(3),r(4),r(5),d);
		}
		else{
			Eigen::VectorXd p = A*c;
			r = b-p;
			d = r.norm();
			if (d<Tol || l==6) {
				break;
			}
		}
		std::tie(Svalue, Spoint) = SuppFun(r, Gi, uFriction);
//		std::cout<<Spoint;
//		printf("\n%.9lf\nSvalue: %.9lf | %d\n",d,Svalue,l);
		Svalue /= d;
	}

	return std::tuple<double, Eigen::Matrix<double,6,1>, Eigen::MatrixXd>(d, r, A);
}

bool ForceClosure::isFC_ZC(SurfacePoint sp1, SurfacePoint sp2, SurfacePoint sp3, SurfacePoint sp4, Eigen::Vector3d cm, double halfAngle)
{
	sp1.position-=cm;
	sp2.position-=cm;
	sp3.position-=cm;
	sp4.position-=cm;
	std::vector<SurfacePoint> Gi{sp1,sp2,sp3,sp4};

	// force-closure test using the ZC distance algorithm (IEEE T-RO'09)
	Eigen::Matrix<double,6,1> wc = -(Wrench(sp1.position,sp1.normal)+Wrench(sp2.position,sp2.normal)+Wrench(sp3.position,sp3.normal)+Wrench(sp4.position,sp4.normal))/4.;
	double Tol=1e-8, d;
	if (wc.norm() < Tol){
		wc << 1,1,1,1,1,1;
	}
	double uFriction=std::tan(halfAngle*M_PI/180.);
//	uFriction=0.17632698070846497540031805328908376395702362060546875;
	std::tie(d, std::ignore, std::ignore) = ZCAlgorithm(wc, Gi, uFriction, Tol);
	return d<Tol;

}

double ForceClosure::getMindist_ZC(SurfacePoint sp1, SurfacePoint sp2, SurfacePoint sp3, SurfacePoint sp4, Eigen::Vector3d cm, double halfAngle)
{
	sp1.position-=cm;
	sp2.position-=cm;
	sp3.position-=cm;
	sp4.position-=cm;
	std::vector<SurfacePoint> Gi{sp1,sp2,sp3,sp4};
	// force-closure test using the ZC distance algorithm (IEEE T-RO'09)
	Eigen::Matrix<double,6,1> wc = -(Wrench(sp1.position,sp1.normal)+Wrench(sp2.position,sp2.normal)+Wrench(sp3.position,sp3.normal)+Wrench(sp4.position,sp4.normal))/4., r;
	double Tol=1e-8, epsilon=1e-5, d;
	if (wc.norm() < Tol){
		wc << 1,1,1,1,1,1;
	}
	double uFriction=std::tan(halfAngle*M_PI/180.);
//	uFriction=0.17632698070846497540031805328908376395702362060546875;
	Eigen::MatrixXd A;
	std::tie(d, r, A) = ZCAlgorithm(wc, Gi, uFriction, Tol);
	if (d > Tol){
		// the grasp does not have force closure
		return 0;
	}
//	Eigen::MatrixXd primW(6,7);
	std::vector<Eigen::Matrix<double,6,1> > primW(7);
	primW.front()=-wc;
	primW[1]=A.col(0);
	primW[2]=A.col(1);
	primW[3]=A.col(2);
	primW[4]=A.col(3);
	primW[5]=A.col(4);
	primW[6]=A.col(5);
//	primW.rightCols<6>()=A;
//	int num_primw = 7;   // number of found primitive wrenches
//	Eigen::MatrixXi CH(6,7);   // facets, each column storing the vertex indices
	std::vector<std::vector<int> > CH(7, std::vector<int>(6));
//	Eigen::MatrixXd norm_facets(6,7);   // facet normals stored as columns
	std::vector<Eigen::Matrix<double,6,1> > norm_facets(7);
	std::vector<double> rdis_facets(7);   // reciprocal of the distance from the origin of each facet
	int max_id;
	double rQual=std::numeric_limits<double>::min();
	for (int k=0;k<7;++k){
		Eigen::Matrix<double, 6, 6> W;
		int at=0;
		for(int i=0;i<7;++i)
			if(i!=k){
				CH[k][at] = i;
				W.col(at++)=primW[i];
			}
		W.transposeInPlace();
		Eigen::Matrix<double, 6, 1> normal = W.colPivHouseholderQr().solve(Eigen::Matrix<double, 6, 1>::Ones());   // use 'sum(pinv(W),1)' for more reliability but less efficiency;
		norm_facets[k] = normal;
		rdis_facets[k] = normal.norm();
		if(rdis_facets[k]>rQual){
			rQual=rdis_facets[k];
			max_id=k;
		}
	}

	//	iterate
	double Svalue;
	Eigen::Matrix<double,6,1> Spoint;
	std::tie(Svalue, Spoint) = SuppFun(norm_facets[max_id],Gi,uFriction);
	double err = Svalue - 1;
	while (err > epsilon*rQual){
//		primW.conservativeResize(Eigen::NoChange, num_primw+1);
//		primW.col(num_primw) = Spoint;
		primW.push_back(Spoint);
		std::vector<int> ids_facets_update;
		for (int k=0;k<norm_facets.size();++k) {
			if (Spoint.dot(norm_facets[k])-1 > 1e-9){
				ids_facets_update.push_back(k);
			}
		}

		for (std::vector<int>::iterator k = ids_facets_update.begin();k!=ids_facets_update.end();++k){
			std::vector<int> facet = CH[*k];
			for (int j=0;j<6;++j){
				std::vector<int> face;
				face.reserve(5);
				for(int i=0;i<6;++i){
					if(i!=j)
						face.push_back(facet[i]);
				}

				// check if the face is a common face shared with any other facet
				std::vector<int>::iterator k2 = ids_facets_update.begin();
				for (;k2!=ids_facets_update.end();++k2){
					if (k2 != k){
						std::vector<int> facet2 = CH[*k2];
						std::vector<int>::iterator i1=face.begin(),i2=facet2.begin();
						while(i1!=face.end() && *i1==*i2){
							++i1;
							++i2;
						}
						++i2;
						while(i1!=face.end() && *i1==*i2){
							++i1;
							++i2;
						}
						if (i1 == face.end()){
							break;
						}
					}
				}
				// face is uncommon with all other facet
				if(k2 == ids_facets_update.end()){
					CH.push_back(face);
					CH.back().reserve(6);
					CH.back().push_back(primW.size()-1);

					Eigen::Matrix<double, 6, 6> W;
					W << primW[face[0]],
					primW[face[1]],
					primW[face[2]],
					primW[face[3]],
					primW[face[4]],
					primW.back();
//					std::cout<<W<<std::endl;

					W.transposeInPlace();
					Eigen::Matrix<double, 6, 1> normal = W.colPivHouseholderQr().solve(Eigen::Matrix<double, 6, 1>::Ones());
					norm_facets.push_back(normal);
					rdis_facets.push_back(normal.norm());
				}
			}
		}
//		for (int i=0; i<rdis_facets.size(); ++i)
//			printf("%lf ",rdis_facets[i]);
//		printf("\n");
		for(std::vector<int>::reverse_iterator k=ids_facets_update.rbegin();k!=ids_facets_update.rend();++k){
			CH.erase(CH.begin() + *k);
			norm_facets.erase(norm_facets.begin() + *k);
			rdis_facets.erase(rdis_facets.begin() + *k);
		}
		max_id=0;
		rQual=rdis_facets.front();
		for (int k=1; k<rdis_facets.size(); ++k) {
			if(rdis_facets[k]>rQual){
				rQual=rdis_facets[k];
				max_id=k;
			}
		}
//		printf("max_id: %d\n",max_id);
		std::tie(Svalue, Spoint) = SuppFun(norm_facets[max_id], Gi, uFriction);
		err = Svalue - 1;
	}
	return 1./rQual;
}
