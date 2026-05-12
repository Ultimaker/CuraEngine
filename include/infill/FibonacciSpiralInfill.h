// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_FIBONACCISPIRAL_H
#define INFILL_FIBONACCISPIRAL_H

#include "geometry/OpenLinesSet.h"
#include "geometry/Shape.h"
#include "utils/Coord_t.h"

namespace cura
{

/*!
 * Generates a Fibonacci-inspired inward spiral infill pattern, designed primarily for use
 * in tree support structures.
 *
 * Concept
 * -------
 * For each closed island in the outline the generator:
 *   1. Walks \p perimeter_start_ratio × perimeter_length along the island boundary to find
 *      the spiral entry point (default 75 % → three quarters around).
 *   2. Computes the AABB center of the island as the spiral center.
 *   3. Generates an inward Archimedean spiral from the entry point toward the center.
 *      The pitch (radial advance per full turn) equals \p line_distance.
 *   4. Shifts the initial spiral angle by (z / line_distance) × 2π so that successive
 *      layers start at a different angle, producing a 3D helix cross-section that creates
 *      inter-layer adhesion.
 *   5. Clips the raw spiral to the island outline.
 *
 * Density control
 * ---------------
 * Tighter winds (higher adhesion) → smaller \p line_distance.
 * Looser winds (easier to remove) → larger \p line_distance.
 *
 * Branch splits / merged circles
 * -------------------------------
 * Each call to generate() receives a single island.  The Infill class calls
 * splitIntoParts() before invoking generateForIsland(), so merged blobs become one
 * island (one centered spiral) and branch splits become two independent islands
 * (two independent spirals) automatically.
 */
class FibonacciSpiralInfill
{
public:
    /*!
     * Generate the spiral infill for a single closed island.
     *
     * \param outline             The closed island to fill (a single Polygon / Shape part).
     * \param line_distance       Radial pitch between successive spiral windings [μm].
     * \param z                   Z coordinate of this layer [μm]; used to shift the start
     *                            angle between layers for 3-D continuity.
     * \param perimeter_start_ratio  Fraction [0,1] of the island perimeter to walk before
     *                               starting the inward spiral.  Default 0.75.
     * \return                    An OpenLinesSet containing the clipped spiral polyline(s).
     */
    [[nodiscard]] static OpenLinesSet generate(
        const Shape& outline,
        coord_t line_distance,
        coord_t z,
        double perimeter_start_ratio = 0.75);

private:
    FibonacciSpiralInfill() = delete;
};

} // namespace cura

#endif // INFILL_FIBONACCISPIRAL_H
