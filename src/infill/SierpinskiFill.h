/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef INFILL_SIERPINSKI_FILL_H
#define INFILL_SIERPINSKI_FILL_H

#include <list>

#include "../utils/polygon.h"
#include "../utils/AABB.h"
#include "../utils/AABB3D.h"

#include "../utils/SVG.h"

namespace cura
{
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
public:
    /*!
     * The type of an edge of a triangle in the space subdivision
     */
    struct Edge
    {
        bool direction; //!< Whether the edge is diagonal, rather than horizontal or vertical
        Point l; //!< The vertex of the edge to the left / inside of the curve
        Point r; //!< The vertex of the edge to the right / outside of the curve
        unsigned int depth; //!< The iteration with which this edge was generated / altered
        Edge (const bool direction, const Point l, const Point r, unsigned int depth) // basic constructor
        : direction(direction)
        , l(l)
        , r(r)
        , depth(depth)
        {
        }
    };

    /*!
     * Basic constructor
     */
    SierpinskiFill(const AABB aabb, int max_depth);

    /*!
     * Generate the cross pattern curve
     */
    Polygon generateCross() const; 

    /*!
     * Generate the Sierpinski space filling curve
     */
    Polygon generateSierpinski() const; 

    /*!
     * Output the edges to a canvas
     */
    void debugOutput(SVG& svg);

protected:
    /*!
     * Process a single step in the recursive fractal
     * \param iteration current recursion depth
     */
    void process(const int iteration);
    
    std::list<Edge> edges; //!< The edges of the triangles of the subdivision which are crossed by the fractal.
    AABB aabb; //!< The square which is the basis of the subdivision of the area on which the curve is based.
    
    const Point pic_size = Point((coord_t)10, (coord_t)10);
    int pic[10][10]
    {{255, 000, 000, 000, 000, 000, 000, 000, 000, 000},
    {225, 000, 255, 000, 000, 000, 000, 000, 000, 000},
    {225, 000, 255, 000, 000, 000, 000, 000, 000, 000},
    {200, 000, 000, 255, 000, 000, 255, 000, 000, 000},
    {175, 000, 000, 000, 000, 000, 000, 000, 000, 255},
    {150, 000, 255, 000, 000, 000, 000, 000, 000, 255},
    {150, 000, 255, 000, 000, 000, 000, 000, 000, 255},
    {125, 000, 000, 000, 000, 000, 000, 000, 255, 000},
    {100, 000, 000, 000, 000, 000, 000, 255, 000, 000},
    { 75,  25, 000, 255, 255, 000, 000, 000, 000, 000}};
};
} // namespace cura


#endif // INFILL_SIERPINSKI_FILL_H
