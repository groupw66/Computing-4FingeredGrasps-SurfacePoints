#ifndef RANGETREE_HPP_INCLUDED
#define RANGETREE_HPP_INCLUDED

#include<vector>
#include<algorithm>
// assume input is already sort (no need to keep id of input before sort)
struct RangeTree{
	std::vector<std::vector<int> > tree;
	std::vector<std::vector<std::vector<int>::iterator> > ptl,ptr;
	std::vector<Point2d> points;

	inline RangeTree(){}

	inline RangeTree(std::vector<Point2d> &points){
		init(points);
	}

	inline void init(std::vector<Point2d> &_points){
		points=_points;
		if(points.empty()){
			// assume that (0,0) can't be in any query
//			points.reserve(1);
//			points.push_back(Point2d(0,0));
			points={Point2d(0,0)};
		}
//		size=1<<(std::max(int(log2(points.size()-1))+2,1)); // 2^(height+1)
		int size=1<<(int(log2(points.size()-1))+2); // 2^(height+1)
//printf("Allocate size: %d\n",size);
//		tree=std::vector<std::vector<int> >(size);
//		ptl=std::vector<std::vector<std::vector<int>::iterator> >(size);
//		ptr=std::vector<std::vector<std::vector<int>::iterator> >(size);
		tree.clear();
		tree.resize(size);
		ptl.clear();
		ptl.resize(size);
		ptr.clear();
		ptr.resize(size);
		makeRangeTree(1,0,points.size());
	}

//	~RangeTree(){
//		while(size--){
////			tree[size].clear();
////			tree[size].shrink_to_fit();
////			ptl[size].clear();
////			ptl[size].shrink_to_fit();
////			ptr[size].clear();
////			ptr[size].shrink_to_fit();
//
//			std::vector<int>().swap(tree[size]);
//			std::vector<std::vector<int>::iterator>().swap(ptl[size]);
//			std::vector<std::vector<int>::iterator>().swap(ptr[size]);
//		}
////		points.clear();
////		points.shrink_to_fit();
//
//		std::vector<Point2d>().swap(points);
//	}

	inline void makeRangeTree(int id,int l,int r){
		if(l+1==r){
//			tree[id].reserve(1);
//			tree[id].push_back(l);
			tree[id]={l};
		}
		else{
			int mid=(l+r)>>1,idl=id<<1,idr=idl+1;
			makeRangeTree(idl,l,mid);
			makeRangeTree(idr,mid,r);
			// merge
			tree[id].reserve(r-l);
			// ptl & ptr are for fractional cascading
			ptl[id].reserve(r-l+1);
			ptr[id].reserve(r-l+1);
//			ptl[id].reserve(r-l);
//			ptr[id].reserve(r-l);
			std::vector<int>::iterator il=tree[idl].begin(),ir=tree[idr].begin();
			while(il!=tree[idl].end() && ir!=tree[idr].end()){
				ptl[id].push_back(il);
				ptr[id].push_back(ir);
				tree[id].push_back(points[*il].second > points[*ir].second ? *ir++ : *il++);
			}
			while(il!=tree[idl].end()){
				ptl[id].push_back(il);
				ptr[id].push_back(ir);
				tree[id].push_back(*il++);
			}
			while(ir!=tree[idr].end()){
				ptl[id].push_back(il);
				ptr[id].push_back(ir);
				tree[id].push_back(*ir++);
			}
			ptl[id].push_back(il);
			ptr[id].push_back(ir);
		}
	}

//	inline bool cmpY(int id, int y){
//		return points[id].second<y;
//	}

	inline std::vector<int> queryId(double minx,double miny,double maxx,double maxy){
//printf("(%lf, %lf) x (%lf, %lf)\n",minx,maxx,miny,maxy);
		std::vector<int> ret;
//for(int i=0;i<points.size();++i) ret.push_back(i); return ret;
		// find split node
		int xid=1,l=0,r=points.size(),mid=r>>1;
		while(l!=mid){
			if(minx>=points[mid].first){
				// go right
				l=mid;
				xid=(xid<<1)+1;
			}
			else if(maxx<=points[mid].first){
				// go left
				r=mid;
				xid<<=1;
			}
			else
				break;
			mid=(l+r)>>1;
		}
		if(l!=mid){
			int yidBegin=std::lower_bound(tree[xid].begin(), tree[xid].end(), miny,
					[&](int id, double y){return points[id].second<y;}) - tree[xid].begin(),
				yidEnd=std::lower_bound(tree[xid].begin(), tree[xid].end(), maxy,
					[&](int id, double y){return points[id].second<y;}) - tree[xid].begin();
//			int yid=lower_bound(tree[xid].begin(), tree[xid].end(), miny, cmpY)-tree[xid].begin();

			int yidBeginR=ptr[xid][yidBegin]-tree[(xid<<1)+1].begin(), yidEndR=ptr[xid][yidEnd]-tree[(xid<<1)+1].begin();
			yidBegin=ptl[xid][yidBegin]-tree[xid<<1].begin();
			yidEnd=ptl[xid][yidEnd]-tree[xid<<1].begin();
			xid<<=1;
			int rl=mid,lr=mid,xidr=xid+1;
			// follow path to left tree
			mid=(l+rl)>>1;

			while(l!=mid){
				//if(minx<=points[mid].first){
				if(minx<points[mid].first){
//printf("\t%d %d %d\n",l,mid,rl);
//printf("\t\t%d %d %d | %d\n",ptr[xid].size(),yidBegin,yidEnd,ptr[xid][yidEnd]-ptr[xid][yidBegin]);
//if(ptr[xid][yidEnd]-ptr[xid][yidBegin]>0)
//printf("\t\t\t%lf %lf\n",points[*ptr[xid][yidBegin]].first,points[*ptr[xid][yidBegin]].second);
					// query right y tree
					ret.insert(ret.end(),ptr[xid][yidBegin],ptr[xid][yidEnd]);

					// go left
					yidBegin=ptl[xid][yidBegin]-tree[xid<<1].begin();
					yidEnd=ptl[xid][yidEnd]-tree[xid<<1].begin();
					rl=mid;
					xid<<=1;
				}
				else{
					// go right
					yidBegin=ptr[xid][yidBegin]-tree[(xid<<1)+1].begin();
					yidEnd=ptr[xid][yidEnd]-tree[(xid<<1)+1].begin();
					l=mid;
					xid=(xid<<1)+1;
				}
				mid=(l+rl)>>1;
			}

			if(minx<points[mid].first && points[mid].first<maxx && miny<points[mid].second && points[mid].second<maxy)
				ret.push_back(mid);

			// follow path to right tree
			mid=(lr+r)>>1;
			while(lr!=mid){
				if(maxx>points[mid].first){
					// query left y tree
					ret.insert(ret.end(),ptl[xidr][yidBeginR],ptl[xidr][yidEndR]);

					// go right
					yidBeginR=ptr[xidr][yidBeginR]-tree[(xidr<<1)+1].begin();
					yidEndR=ptr[xidr][yidEndR]-tree[(xidr<<1)+1].begin();
					lr=mid;
					xidr=(xidr<<1)+1;
				}
				else{
					// go left
					yidBeginR=ptl[xidr][yidBeginR]-tree[xidr<<1].begin();
					yidEndR=ptl[xidr][yidEndR]-tree[xidr<<1].begin();
					r=mid;
					xidr<<=1;
				}
				mid=(lr+r)>>1;
			}
		}
		if(minx<points[mid].first && points[mid].first<maxx && miny<points[mid].second && points[mid].second<maxy)
			ret.push_back(mid);
// check if all ret are in correct range
for(auto id:ret){
if(points[id].first<=minx || points[id].first>=maxx || points[id].second<=miny || points[id].second>=maxy)
printf("Range Tree Error(output not in range)\n");
}
		return ret;
	}
};

#endif // RANGETREE_HPP_INCLUDED
