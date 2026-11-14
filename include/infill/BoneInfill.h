// Copyright (c) 2025 UltiMaker and contributors
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_BONEINFILL_H
#define INFILL_BONEINFILL_H

#include <vector>

#include "geometry/Point2LL.h"
#include "utils/Coord_t.h"

namespace cura
{

class AABB;
class AngleDegrees;
class OpenLinesSet;
class Shape;

/*!
 * Settings that shape the bone-like infill.
 */
struct BoneParameters
{
    int seed{ 1 }; //!< Selects one of the many possible structures.
    double irregularity{ 0.4 }; //!< 0 to 1. How much the thickness and spacing of the struts vary; 0 gives a regular, almost periodic structure.
    double alignment{ 0.6 }; //!< 0 to 1. How strongly the struts follow one main direction, like trabeculae follow the load in real bone.
    double tilt{ 0.0 }; //!< Degrees. Tilt of the main direction away from vertical.
    double azimuth{ 0.0 }; //!< Degrees. Direction in the XY plane the main direction is tilted towards.
    double connectivity{ 1.0 }; //!< 0 to 1. 1 gives one fully connected network of plates; lower values split it into separate cells and rods.
    coord_t cortical_width{
        3000
    }; //!< Width of the zone along the walls where the structure is made denser, like the transition to the hard outer shell of a bone. 0 turns it off.
    int cortical_lines{ 1 }; //!< How many extra parallel struts on each side of every strut in that zone.
};

/*!
 * Infill that mimics the structure of spongy (trabecular) bone.
 *
 * Trabecular bone is a connected network of thin plates and rods that lines up with the main load direction and gets
 * denser towards the hard outer shell. This is imitated with a smooth random 3D field, a sum of many cosine waves
 * ("spinodoid" structure): the infill follows the surface where the field crosses a fixed level, which gives a
 * bicontinuous, organic network of plates. The directions of the waves decide how the structure is oriented: waves
 * mostly perpendicular to the main direction give struts running along it. Because the field is continuous, the
 * pattern changes smoothly from layer to layer, so each layer is supported by the one below it.
 */
class BoneInfill
{
public:
    explicit BoneInfill(const BoneParameters& parameters);

    /*!
     * Generate the infill in the given outline.
     * @param[out] result_polylines The resulting infill lines.
     * @param[out] result_polygons The resulting closed loops, if there are any.
     * @param line_distance Average distance between neighbouring struts in a layer, which sets the density.
     * @param in_outline The area to fill.
     * @param z The Z coordinate of this layer, which makes the pattern vary between layers.
     * @param line_width The line width at which the infill will be printed.
     * @param rotation Rotation of the whole structure.
     */
    void generateInfill(
        OpenLinesSet& result_polylines,
        Shape& result_polygons,
        const coord_t line_distance,
        const Shape& in_outline,
        const coord_t z,
        const coord_t line_width,
        const AngleDegrees& rotation) const;

private:
    struct Wave
    {
        double kx, ky, kz; //!< Wave vector, rad per mm, for a structure with a line distance of 1 mm.
        double phase;
    };

    /*! Values of the field on a regular grid, for one layer. */
    struct Grid
    {
        double x0, y0; //!< Position of the first node, in mm.
        double cell; //!< Distance between nodes, in mm.
        size_t nx, ny;
        std::vector<double> values;
    };

    /*! Calculate the field for one layer on a grid that covers the given area. */
    Grid evaluate(double scale, const AABB& area, double z_mm, double cell_mm) const;

    /*! The curves where the field crosses the given level, as polylines (in microns). */
    static std::vector<std::vector<Point2LL>> trace(const Grid& grid, double level);

    BoneParameters parameters_;
    std::vector<Wave> waves_; //!< The waves, for a line distance of 1 mm.
    double gradient_strength_; //!< Mean size of the gradient of the field along a layer, for the waves above.
};

} // namespace cura

#endif // INFILL_BONEINFILL_H
