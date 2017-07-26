/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef INFILL_SPACE_FILLING_TREE_FILL_H
#define INFILL_SPACE_FILLING_TREE_FILL_H

#include "../utils/polygon.h"
#include "../utils/AABB.h"
#include "../utils/AABB3D.h"
#include "../utils/SpaceFillingTree.h"

#include "../utils/SVG.h"

namespace cura
{
/*!
 * A class for generating the Cross and Cross 3D infill patterns.
 * 
 * The generated line can be seen as an offset from a fractal cross,
 * such that the offsetted line is in between the cross fractal and the square area which is the limit of the fractal.
 * 
 * This class is a generator for one model.
 * It should be reused through multiple layers of the same \ref SliceMeshStorage
 * 
 * The pattern looks like this but 45degrees rotated:
 *               ▄▉▄        white cross left out
 *             ▄  ▉  ▄    ↙  by the black, a.k.a. anti-cross
 *            ▀▉▀▀▉▀▀▉▀              ....................
 *         ▄▉▄   ▄▉▄   ▄▉▄           :░░░░░░░░░░░░░░░░░░:
 *       ▄  ▉  ▄  ▉  ▄  ▉  ▄         :░░░+-----+░░░+----:
 *      ▀▉▀▀▉▀▀▉▀▀▉▀▀▉▀▀▉▀▀▉▀:       :░░░|     |░░░|    :
 *         ▄▉▄   ▄▉▄   ▄▉▄   : ===>  :░░░+--.   'v'     :
 *          ▀  ▄  ▉  ▄  ▀....:       :░░░░░░░>          :
 *            ▀▉▀▀▉▀▀▉▀              :░░░+--'           :
 *               ▄▉▄                 :...|..............:
 *                ▀                   note how the anti-crosses have a diagonal straight part,
 *                                    but end differently than the black crosses.
 *                                    While the black crosses end in a point,
 *                                    the anti-crosses end in a square.
 */
class SpaceFillingTreeFill
{
    /*!
     * The parameters to pass to the constructor of the \ref SpaceFillingTreeFill::tree
     */
    struct TreeParams
    {
        Point middle; //!< The middle of where the grid should be centered
        coord_t radius; //!< The distance from the middle to the side / top of the square which is filled with the pattern
        int depth; //!< The recursion depth of the tree fractal
    };
public:
    /*!
     * Constructor for constructing an infill pattern generator.
     * 
     * \param line_distance The width of the crosses and the width of the anti-crosses.
     * \param model_aabb The aabb to at aleast cover with the infill pattern
     */
    SpaceFillingTreeFill(coord_t line_distance, AABB3D model_aabb);

    /*!
     * Generate the polygons or line segments of the cross pattern.
     * The pattern is cut of by the \p outlines.
     * These outlines should be smaller when we are zig zaggifying the infill.
     * 
     * \param outlines The area by which to cut the infill pattern.
     * \param shift The shift of the infill pattern from the cross fractal toward the square region
     * \param zig_zaggify Whether to connect the cross lines via the \p outlines
     * \param fill_angle The direction of the main crosses (modulo 90degrees)
     * \param[out] result_polygons The output when \p zig_zaggify
     * \param[out] result_lines The output when not \p zig_zaggify
     */
    void generate(const Polygons& outlines, coord_t shift, bool zig_zaggify, double fill_angle, Polygons& result_polygons, Polygons& result_lines) const;
private:
    AABB3D model_aabb; //!< The AABB of the boundary to cover
    coord_t line_distance; //!< The width of the crosses and of the straight part of the anti-crosses
    TreeParams tree_params; //!< The parameters with which to construct the \p tree
    SpaceFillingTree tree; //!< The space filling tree cross fractal

    /*!
     * Compute the parameters with which to construct the tree.
     * 
     * The is done in a separate function so that those parameters can be calculated during the construction of this object
     * 
     * \param line_distance The width of the crosses and of the straight parts of the anti-crosses
     * \param model_aabb The aabb to at aleast cover with the infill pattern
     * \return The parameters with which to construct the tree.
     */
    static TreeParams getTreeParams(coord_t line_distance, AABB3D model_aabb);

    /*!
     * Generate the pattern itself by doing a depth first walk over the fractal.
     * 
     * The parent nodes are present in this path multiple times.
     * 
     * \param[out] path The path to generate
     */
    void generateTreePath(PolygonRef path) const;

    /*!
     * Generate the line in between the tree path and the circumscribed
     * square of each stage in the fractal.
     * 
     * \param path the tree path which walks along the cross fractal tree
     * \param offset The offset from the cross fractal on straight pieces
     * \return The cross infill pattern which isn't bounded to the outlines yet
     */
    Polygon offsetTreePath(const ConstPolygonRef path, coord_t offset) const;
};
} // namespace cura


#endif // INFILL_SPACE_FILLING_TREE_FILL_H
