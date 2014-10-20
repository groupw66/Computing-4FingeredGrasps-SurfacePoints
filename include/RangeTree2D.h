#ifndef RANGETREE2D_H
#define RANGETREE2D_H

#include <Eigen/Dense>
#include<vector>
#include<algorithm>

class RangeTree2D
{
    public:
        std::vector<std::vector<int> > tree;
        std::vector<std::vector<std::vector<int>::iterator> > ptl,ptr;
        std::vector<Eigen::Vector2d> points;
        RangeTree2D();
        virtual ~RangeTree2D();
        inline RangeTree2D(const std::vector<Eigen::Vector2d> &points){
            init(points);
        }
        void init(const std::vector<Eigen::Vector2d> &_points);
        void makeRangeTree(int id,int l,int r);
        std::vector<int> queryId(double minx,double miny,double maxx,double maxy);
    protected:
    private:
};

#endif // RANGETREE2D_H
