#include "RangeTree2D.h"

RangeTree2D::RangeTree2D()
{
    //ctor
}

RangeTree2D::~RangeTree2D()
{
    //dtor
}

// assume input is already sort (no need to keep id of input before sort)
void RangeTree2D::init(const std::vector<Eigen::Vector2d>& _points)
{
    points = _points;
    if(points.empty()){
        // assume that (0,0) can't be in any query
        points={Eigen::Vector2d(0,0)};
    }
    int size = 1 << (int(std::log2(points.size()-1))+2); // 2^(height+1)
    tree.clear();
    tree.resize(size);
    ptl.clear();
    ptl.resize(size);
    ptr.clear();
    ptr.resize(size);
    makeRangeTree(1, 0, points.size() );
}
void RangeTree2D::makeRangeTree(int id, int l, int r)
{
    if(l+1==r){
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
        std::vector<int>::iterator il = tree[idl].begin(), ir = tree[idr].begin();
        while( il!=tree[idl].end() && ir!=tree[idr].end() ){
            ptl[id].push_back(il);
            ptr[id].push_back(ir);
            tree[id].push_back(points[*il].y() > points[*ir].y() ? *ir++ : *il++);
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

std::vector<int> RangeTree2D::queryId(double minx, double miny, double maxx, double maxy)
{
    std::vector<int> ret;
    // find split node
    int xid=1,l=0,r=points.size(),mid=r>>1;
    while(l!=mid){
        if(minx>=points[mid].x()){
            // go right
            l=mid;
            xid=(xid<<1)+1;
        }
        else if(maxx<=points[mid].x()){
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
                [&](int id, double y){return points[id].y()<y;}) - tree[xid].begin(),
            yidEnd=std::lower_bound(tree[xid].begin(), tree[xid].end(), maxy,
                [&](int id, double y){return points[id].y()<y;}) - tree[xid].begin();
//			int yid=lower_bound(tree[xid].begin(), tree[xid].end(), miny, cmpY)-tree[xid].begin();

        int yidBeginR=ptr[xid][yidBegin]-tree[(xid<<1)+1].begin(), yidEndR=ptr[xid][yidEnd]-tree[(xid<<1)+1].begin();
        yidBegin=ptl[xid][yidBegin]-tree[xid<<1].begin();
        yidEnd=ptl[xid][yidEnd]-tree[xid<<1].begin();
        xid<<=1;
        int rl=mid,lr=mid,xidr=xid+1;
        // follow path to left tree
        mid=(l+rl)>>1;

        while(l!=mid){
            if(minx<points[mid].x()){
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

        if(minx<points[mid].x() && points[mid].x()<maxx && miny<points[mid].y() && points[mid].y()<maxy)
            ret.push_back(mid);
        // follow path to right tree
        mid=(lr+r)>>1;
        while(lr!=mid){
            if(maxx>points[mid].x()){
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
    if(minx<points[mid].x() && points[mid].x()<maxx && miny<points[mid].y() && points[mid].y()<maxy)
        ret.push_back(mid);
    //check if all ret are in correct range
    for(auto id:ret){
        if(points[id].x()<=minx || points[id].x()>=maxx || points[id].y()<=miny || points[id].y()>=maxy)
            printf("Range Tree Error(output not in range)\n");
    }
    return ret;
}

