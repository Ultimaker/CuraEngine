/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef INFILL_SIERPINSKI_FILL_H
#define INFILL_SIERPINSKI_FILL_H

#include <list>

#include "../utils/polygon.h"
#include "../utils/AABB.h"
#include "../utils/AABB3D.h"

#include "../utils/SVG.h"

#include "DensityProvider.h"


namespace cura
{
    
class SierpinskiFillTest; 
/*!
 * A class for generating the Cross and Cross 3D infill patterns.
 * 
 * The line is generated from a recurvive subdivision of the area of a square into triangles.
 *  _______    _______    _______    _______
 * |      /|  |\     /|  |\  |  /|  |\ /|\ /|          .
 * |    /  |  |  \ /  |  |__\|/__|  |/_\|/_\|  etc     .
 * |  /    |  |  / \  |  |  /|\  |  |\ /|\ /|          .
 * |/______|  |/_____\|  |/__|__\|  |/_\|/_\|          .
 * 
 * This subdivision is performed by subdividing the triangles with diagonal (D) lines and straight (S) lines alternatingly.
 * 
 * These are stored as the sequence of edges which will be crossed by the space filling fractal.
 * Each trangle is stored as two consecutive edges.
 * Each two consecutive edges share one end point.
 * 
 * Not all edges in the square are subdivision are stored, because not all edges are crossed by the fractal:
 * 
 * |          /  |\         /   \    |    /    \   /|\   /    \ | / \ | / 
 * |        /    |  \     /       \  |  /        X  |  X     ---X--+--X---
 * |      /      |    \ /       ____\|/_____   /____|____\_   / |__|__| \ 
 * |    /        |      \      |     |\       |\    |    /    \ |  |  | / 
 * |  /          |        \    |     |  \     |  \  |  X     +--\--+--X---
 * |/_________   |__________\  |_____|    \   |____\|/   \   |__| \ / | \ 
 * 
 * 
 * The rules for generating the edges can be summarizd as follows:
 * D step:
 * SDS -> SDDDS (omit the inner half of the original D)
 * SS -> SDS
 * 
 * S step:
 * DSD -> DSSSD (omit the outer half of the original S)
 * DD -> DSD
 * 
 * 
 * 
 * By following the path along the middle points of each triangle the Siepinski space filling curve will be generated.
 * By following the path along the middle of each edge the cross space filling curve will be generated.
 */
class SierpinskiFill
{
    friend class SierpinskiFillTest;
public:
    /*!
     * Basic constructor
     */
    SierpinskiFill(const DensityProvider& density_provider, const AABB aabb, int max_depth, const coord_t line_width, bool dithering);

    ~SierpinskiFill();

    /*!
     * Generate the cross pattern curve
     */
    Polygon generateCross() const; 

    /*!
     * Generate the Sierpinski space filling curve
     */
    Polygon generateSierpinski() const; 

    /*!
     * Generate the Sierpinski space filling curve
     */
    Polygon generateCross(coord_t z, coord_t min_dist_to_side) const; 

    /*!
     * Output the edges to a canvas
     */
    void debugOutput(SVG& svg);

protected:
    struct Edge
    {
        Point l, r;
    };
    struct SierpinskiTriangle
    {
        enum class SierpinskiDirection
        {
            AC_TO_AB,
            AC_TO_BC,
            AB_TO_BC
        };
        Point straight_corner;
        Point a;
        Point b;
        SierpinskiDirection dir;
        bool straight_corner_is_left;
        int depth;
        
        float area;
        float requested_length;
        float realized_length;
        float total_child_realized_length;
        float error_left;
        float error_right;

        SierpinskiTriangle(Point straight_corner, Point a, Point b, SierpinskiDirection dir, bool straight_corner_is_left, int depth)
        : straight_corner(straight_corner)
        , a(a)
        , b(b)
        , dir(dir)
        , straight_corner_is_left(straight_corner_is_left)
        , depth(depth)
        , requested_length(0)
        , total_child_realized_length(0)
        , error_left(0)
        , error_right(0)
        {}
        SierpinskiTriangle()
        : straight_corner(no_point)
        , a(no_point)
        , b(no_point)
        , depth(0)
        , requested_length(0)
        , total_child_realized_length(0)
        , error_left(0)
        , error_right(0)
        {}
        bool isValid()
        {
            return straight_corner != no_point;
        }
        Edge getFromEdge()
        {
            Edge ret;
            switch(dir)
            {
                case SierpinskiDirection::AB_TO_BC:
                    ret = Edge{a, b};
                    break;
                case SierpinskiDirection::AC_TO_AB:
                    ret = Edge{straight_corner, a};
                    break;
                case SierpinskiDirection::AC_TO_BC:
                    ret = Edge{straight_corner, a};
                    break;
            }
            if (!straight_corner_is_left)
            {
                std::swap(ret.l, ret.r);
            }
            return ret;
        }
        Edge getToEdge()
        {
            Edge ret;
            switch(dir)
            {
                case SierpinskiDirection::AB_TO_BC:
                    ret = Edge{straight_corner, b};
                    break;
                case SierpinskiDirection::AC_TO_AB:
                    ret = Edge{b, a};
                    break;
                case SierpinskiDirection::AC_TO_BC:
                    ret = Edge{straight_corner, b};
                    break;
            }
            if (!straight_corner_is_left)
            {
                std::swap(ret.l, ret.r);
            }
            return ret;
        }
        
        float getTotalError()
        {
            return error_left + error_right;
        }
        float getErroredValue()
        {
            return requested_length + getTotalError(); 
        }
        float getSubdivisionError()
        {
            return getErroredValue() - total_child_realized_length;
        }
        float getValueError()
        {
            return getErroredValue() - realized_length;
        }
        
        std::vector<SierpinskiTriangle> children;
    };


    
    bool dithering;
    bool constraint_error_diffusion; //!< Whether to diffuse errors caused by constraints between consecutive cells
    
    bool use_errors_in_dithering = true;
    
    
    const DensityProvider& density_provider; //!< function which determines the requested infill density of a triangle defined by two consecutive edges.
    AABB aabb; //!< The square which is the basis of the subdivision of the area on which the curve is based.
    coord_t line_width; //!< The line width of the fill lines
    int max_depth; //!< Maximum recursion depth of the fractal

    size_t pre_division_tree_size;
    
    SierpinskiTriangle root;

    std::list<SierpinskiTriangle*> sequence; //!< The triangles of the subdivision which are crossed by the fractal.

    

    void createTree();
    void createTree(SierpinskiTriangle& sub_root);
    void createTreeStatistics(SierpinskiTriangle& sub_root);
    void createTreeRequestedLengths(SierpinskiTriangle& sub_root);
    
    void createLowerBoundSequence();
    
    /*!
     * Subdivide all nodes if possible.
     * 
     * Start trying cells with lower recursion level before trying cells with deeper recursion, i.e. higher density value.
     * 
     * \return Whether the sequence has changed.
     */
    bool subdivideAll();

    /*!
     * Bubble up errors from nodes which like to subdivide more,
     * but which are constrained by neighboring cells of lower recursion level.
     * 
     * \return Whether we have redistributed errors which could cause a new subdivision 
     */
    bool bubbleUpConstraintErrors();

    /*!
     * Subdivide a node into its children.
     * Redistribute leftover errors needed for this subdivision and account for errors needed to keep the children balanced.
     * 
     * \param it iterator to the node to subdivide
     * \param redistribute_errors Whether to redistribute the accumulated errors to neighboring nodes and/or among children
     * \return The last child, so that we can iterate further through the sequence on the input iterator.
     */
    std::list<SierpinskiTriangle*>::iterator subdivide(std::list<SierpinskiTriangle*>::iterator begin, std::list<SierpinskiTriangle*>::iterator end, bool redistribute_errors);


    /*!
     * Redistribute positive errors in as much as they aren't needed to subdivide this node.
     * If this node has received too much positive error then it will subdivide
     * and pass along the error from whence it came.
     * 
     * This is called just before performing a subdivision.
     */
    void redistributeLeftoverErrors(std::list<SierpinskiTriangle*>::iterator begin, std::list<SierpinskiTriangle*>::iterator end);

    /*!
     * Balance child values such that they account for the minimum value of their recursion level.
     * 
     * Account for errors caused by unbalanced children.
     * Plain subdivision can lead to one child having a smaller value than the density_value associated with the recursion depth
     * if another child has a high enough value such that the parent value will cause subdivision.
     * 
     * In order to compensate for the error incurred, we more error value from the high child to the low child
     * such that the low child has an erroredValue of at least the density_value associated with the recusion depth.
     * 
     * \param node The parent node of the children to balance
     */
    void balanceErrors(std::list<SierpinskiTriangle*>::iterator begin, std::list<SierpinskiTriangle*>::iterator end);

    void diffuseError();

    bool isConstrainedBackward(std::list<SierpinskiTriangle*>::iterator it);
    bool isConstrainedForward(std::list<SierpinskiTriangle*>::iterator it);
    
    float getSubdivisionError(std::list<SierpinskiTriangle*>::iterator begin, std::list<SierpinskiTriangle*>::iterator end);

    void debugCheck(bool check_subdivision = true);
};
} // namespace cura


#endif // INFILL_SIERPINSKI_FILL_H
