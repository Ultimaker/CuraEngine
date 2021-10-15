//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "../utils/Coord_t.h"

namespace cura
{

class Polygons;

class GyroidInfill
{
public:
    GyroidInfill();

    ~GyroidInfill();

    /*!
     * Generate the Gyroid infill pattern within a certain outline.
     *
     * This is a 3D infill pattern which has one single permeable volume (excellent for casting or soluble support),
     * very consistent low stiffness in all directions. Fairly high toughness due to the low stiffness.
     *
     * The pattern generates a sine-wave in two directions that varies across the Z coordinate, gradually changing
     * between the X and Y directions. This is a 2D pattern, but by supplying a Z coordinate the pattern will vary
     * across different heights, producing a 3D pattern.
     * \param result_lines Output variable to store the resulting polyline segments in.
     * \param zig_zaggify Whether to connect the polylines at their endpoints, forming one single polyline or at least
     * very few interruptions in the material flow.
     * \param line_distance Distance between adjacent curves. This determines the density of the pattern (when printed
     * at a fixed line width).
     * \param in_outline The outline in which to print the pattern. The input shape, so to say.
     * \param z The Z coordinate of this layer. Different Z coordinates cause the pattern to vary, producing a 3D
     * pattern.
     */
    static void generateTotalGyroidInfill(Polygons& result_lines, bool zig_zaggify, coord_t line_distance, const Polygons& in_outline, coord_t z);
    
private:

};

} // namespace cura

