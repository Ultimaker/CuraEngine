/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef UTILS_R_STAR_TREE_2D_H
#define UTILS_R_STAR_TREE_2D_H


#include "r-star-tree/RStarTree.h"
#include "AABB.h"

#define PARENT_TREE_TYPE RStarTree<LeafType, 2, 2, 4>

namespace cura
{



template <typename LeafType>
class RStarTree2D : PARENT_TREE_TYPE
{
    typedef PARENT_TREE_TYPE RTree;
public:
    RStarTree2D()
    : PARENT_TREE_TYPE()
    {
    }

    void insert(const LeafType item, AABB aabb);

    std::vector<LeafType> findEnclosing(const Point location) const;
protected:
    /*!
     * returns true if the compared boundary encloses the given point
     */
    struct AcceptEnclosingPoint
    {
        const Point& point;
        explicit AcceptEnclosingPoint(const Point& point)
        : point(point)
        {}
        
        bool operator()(const typename RTree::Node* const node) const 
        {
            const typename PARENT_TREE_TYPE::BoundingBox& bb = node->bound;
            return bb.edges[0].first <= point.X && bb.edges[0].second >= point.X
                && bb.edges[1].first <= point.Y && bb.edges[1].second >= point.Y;
        }
        
        bool operator()(const typename RTree::Leaf* const leaf) const 
        { 
            const typename PARENT_TREE_TYPE::BoundingBox& bb = leaf->bound;
            return bb.edges[0].first <= point.X && bb.edges[0].second >= point.X
                && bb.edges[1].first <= point.Y && bb.edges[1].second >= point.Y;
        }
        
        private: AcceptEnclosingPoint(){}
    };
    struct Visitor {
        std::vector<LeafType> ret;
        bool ContinueVisiting;

        Visitor()
        : ContinueVisiting(true)
        {};

        void operator()(const typename PARENT_TREE_TYPE::Leaf* const leaf) 
        {
            ret.push_back(leaf->leaf);
        }
    };
};

template <typename LeafType>
void RStarTree2D<LeafType>::insert(const LeafType item, AABB aabb)
{
    typename RTree::BoundingBox bb;

    bb.edges[0].first  = aabb.min.X;
    bb.edges[0].second = aabb.max.X;
    
    bb.edges[1].first  = aabb.min.Y;
    bb.edges[1].second = aabb.max.Y;
    PARENT_TREE_TYPE::Insert(item, bb);
}

template <typename LeafType>
std::vector<LeafType> RStarTree2D<LeafType>::findEnclosing(const Point location) const
{

    RStarTree2D<LeafType>& non_const_this = *const_cast<RStarTree2D<LeafType>*>(this);
    Visitor visitor = non_const_this.Query(RStarTree2D::AcceptEnclosingPoint(location), Visitor());
    return visitor.ret;
}


}//namespace cura

#endif//UTILS_R_STAR_TREE_2D_H

