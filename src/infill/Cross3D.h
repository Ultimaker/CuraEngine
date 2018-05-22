/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_CROSS_3D_H
#define INFILL_CROSS_3D_H

#include <array>
#include <list>

#include "../utils/optional.h"

#include "../utils/IntPoint.h"
#include "../utils/AABB3D.h"

#include "DensityProvider.h"

namespace cura
{

class Cross3DTest; 
/*!
 * 
 * Triangles are subdivided into two children like so:
 * |\       |\        .
 * |A \     |A \      .
 * |    \   |    \    . where S is always the 90* straight corner
 * |     S\ |S____B\  .       The direction between A and B is maintained
 * |      / |S    A/
 * |    /   |    /      Note that the polygon direction flips between clockwise and SSW each subdivision
 * |B /     |B /
 * |/       |/
 * 
 * The direction of the space filling curve along each triangle is recorded:
 * 
 * |\                           |\                                        .
 * |B \  AS_TO_BS               |B \   AS_TO_AB                           .
 * |  ↑ \                       |  ↑ \             is_expanding remains   .
 * |  ↑  S\  subdivides into    |S_↑__A\                                  .
 * |  ↑   /                     |S ↑  B/                                  .
 * |  ↑ /                       |  ↑ /             is_expanding remains   .
 * |A /                         |A /   AB_TO_BS                           .
 * |/                           |/                                        .
 *                                                                        .
 * |\                           |\                                        .
 * |B \  AS_TO_AB               |B \   AS_TO_BS                           .
 * |    \                       |↖   \             is_expanding remains   .
 * |↖    S\  subdivides into    |S_↖__A\                                  .
 * |  ↖   /                     |S ↑  B/                                  .
 * |    /                       |  ↑ /             is_expanding flips     .
 * |A /                         |A /   AB_TO_BS                           .
 * |/                           |/                                        .
 *                                                                        .
 * |\                           |\                                        .
 * |B \  AB_TO_BS               |B \   AS_TO_AB                           .
 * |  ↗ \                       |  ↑ \            is_expanding flips      .
 * |↗    S\  subdivides into    |S_↑__A\                                  .
 * |      /                     |S ↗  B/                                  .
 * |    /                       |↗   /            is_expanding remains    .
 * |A /                         |A /   AS_TO_BS                           .
 * |/                           |/                                        .
 * 
 */
class Cross3D
{
    friend class Cross3DTest;
    using idx_t = int_fast32_t;
public:

    Cross3D(const DensityProvider& density_provider, const AABB3D aabb, const int max_depth, coord_t line_width);

protected:
    static constexpr char max_subdivision_count = 4; //!< Prisms are subdivided into 2 or 4 prisms
    static constexpr char number_of_sides = 4; //!< Prisms connect above, below and before and after

    struct Triangle
    {
        /*!
         * The order in
         * Which the edges of the triangle are crossed by the Sierpinski curve.
         */
        enum class Direction
        {
            AC_TO_AB,
            AC_TO_BC,
            AB_TO_BC
        };
        Point straight_corner; //!< C, the 90* corner of the triangle
        Point a; //!< The corner closer to the start of the space filling curve
        Point b; //!< The corner closer to the end of the space filling curve
        Direction dir; //!< The (order in which) edges being crossed by the Sierpinski curve.
        bool straight_corner_is_left; //!< Whether the \ref straight_corner is left of the curve, rather than right. I.e. whether triangle ABC is counter-clockwise

        std::array<Triangle, 2> subdivide() const;
    };
    struct Prism
    {
        Triangle triangle;
        coord_t z_min; //!< The starting Z
        coord_t z_max; //!< The ending Z
        bool is_expanding; //!< Whether the surface patch to fill this prism has one vertex pointing downward rather than two.

        //! simple constructor
        Prism(
            Triangle triangle,
            coord_t z_min,
            coord_t z_max,
            bool is_expanding
        )
        : triangle(triangle)
        , z_min(z_min)
        , z_max(z_max)
        , is_expanding(is_expanding)
        {}

        //! initialize with invalid data
        Prism()
        {}

        bool isHalfCube();
        bool isQuarterCube();
    };
    struct Cell; // forward decl
    struct Link;
    using LinkIterator = typename std::list<Link>::iterator;
    struct Link
    {
        idx_t to_index;
        std::optional<LinkIterator> reverse; //!< The link in the inverse direction
        float loan; //!< amount of requested_filled_area loaned from one cell to another, when subdivision of the former is prevented by the latter. This value should always be positive.
        idx_t from_index()
        {
            assert(reverse);
            return (*reverse)->to_index;
        }
        Link& getReverse() const
        {
            assert(reverse);
            return **reverse;
        }
        Link(idx_t to_index)
        : to_index(to_index)
        , loan(0.0)
        {}
    };
    struct Cell
    {
        Prism prism;
        idx_t index; //!< index into \ref Cross3D::cell_data
        char depth; //!< recursion depth
        float volume; //!< The volume of the prism in mm^3
        float filled_volume_allowance; //!< The volume to be filled corresponding to the average density requested by the volumetric density specification.
        float minimally_required_density; //!< The largest required density across this area. For when the density specification is the minimal density at each locatoin.
        bool is_subdivided;
        std::vector<std::list<Link>> adjacent_cells; //!< the adjacent cells for each edge/face of this cell

        std::array<idx_t, max_subdivision_count> children;

        Cell(const Prism& prism, const idx_t index, const int depth)
        : prism(prism)
        , index(index)
        , depth(depth)
        , volume(-1)
        , filled_volume_allowance(0)
        , minimally_required_density(-1)
        , is_subdivided(false)
        , adjacent_cells(number_of_sides)
        {
//             children.fill(-1); --> done in createTree(...)
        }

        uint_fast8_t getChildCount();
    };

    std::vector<Cell> cell_data; //!< Storage for all the cells. The data of a binary/quaternary tree in depth first order.

    AABB3D aabb;
    int max_depth;
    coord_t line_width; //!< The line width of the fill lines

    const DensityProvider& density_provider; //!< function which determines the requested infill density of a triangle defined by two consecutive edges.

    /*!
     * Initialize the the tree structure from the density specification
     */ 
    void initialize();

    float getDensity(const Cell& cell) const;

    
private:
    void createTree();
    void createTree(Cell& sub_tree_root, int max_depth);
    void setVolume(Cell& sub_tree_root);
    void setSpecificationAllowance(Cell& sub_tree_root);

};

} // namespace cura


#endif // INFILL_CROSS_3D_H
