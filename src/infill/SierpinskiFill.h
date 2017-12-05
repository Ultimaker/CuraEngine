/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef INFILL_SIERPINSKI_FILL_H
#define INFILL_SIERPINSKI_FILL_H

#include <list>

#include "../utils/polygon.h"
#include "../utils/AABB.h"
#include "../utils/AABB3D.h"

#include "../utils/SVG.h"

#include "../utils/FingerTree.h"

#include "SierpinskiFillEdge.h"
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
        
        float total_value;
        float density_value;
        float density_color;
        float error_left;
        float error_right;

        SierpinskiTriangle(Point straight_corner, Point a, Point b, SierpinskiDirection dir, bool straight_corner_is_left, int depth)
        : straight_corner(straight_corner)
        , a(a)
        , b(b)
        , dir(dir)
        , straight_corner_is_left(straight_corner_is_left)
        , depth(depth)
        , total_value(0)
        , error_left(0)
        , error_right(0)
        {}
        SierpinskiTriangle()
        : straight_corner(no_point)
        , a(no_point)
        , b(no_point)
        , depth(0)
        , total_value(0)
        , error_left(0)
        , error_right(0)
        {}
        bool isValid()
        {
            return straight_corner != no_point;
        }
    };

    bool dithering;
    const DensityProvider& density_provider; //!< function which determines the requested infill density of a triangle defined by two consecutive edges.
    AABB aabb; //!< The square which is the basis of the subdivision of the area on which the curve is based.
    coord_t line_width; //!< The line width of the fill lines
    int max_depth; //!< Maximum recursion depth of the fractal

    size_t pre_division_tree_size;
    FingerTree<SierpinskiTriangle> pre_division_tree; //!< Tree with fully subdivided triangles to get accurate desnity estimated for each possible triangle.
    
    size_t calculatePreDivisionTreeSize(unsigned int max_depth);

    std::list<SierpinskiFillEdge> edges; //!< The edges of the triangles of the subdivision which are crossed by the fractal.

    /*!
     * Process a single step in the recursive fractal
     * \param iteration current recursion depth
     */
    void process(const int iteration);
};
} // namespace cura


#endif // INFILL_SIERPINSKI_FILL_H
