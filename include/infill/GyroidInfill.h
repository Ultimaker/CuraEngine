// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_GYROIDINFILL_H
#define INFILL_GYROIDINFILL_H

#include "infill/AbstractLinesInfill.h"

namespace cura
{

class GyroidInfill : public AbstractLinesInfill
{
public:
    GyroidInfill() = default;

    ~GyroidInfill() override = default;

protected:
    /*!
     * Generate the Gyroid infill pattern within a certain outline.
     *
     * This is a 3D infill pattern which has one single permeable volume (excellent for casting or soluble support),
     * very consistent low stiffness in all directions. Fairly high toughness due to the low stiffness.
     *
     * The pattern generates a sine-wave in two directions that varies across the Z coordinate, gradually changing
     * between the X and Y directions. This is a 2D pattern, but by supplying a Z coordinate the pattern will vary
     * across different heights, producing a 3D pattern.
     * \param line_distance Distance between adjacent curves. This determines the density of the pattern (when printed
     * at a fixed line width).
     * \param in_outline The outline in which to print the pattern. The input shape, so to say.
     * \param z The Z coordinate of this layer. Different Z coordinates cause the pattern to vary, producing a 3D
     * pattern.
     * \param line_width Unused in this context.
     * \return The list of raw gyroid lines.
     */
    OpenLinesSet generateParallelLines(const coord_t line_distance, const Shape& in_outline, const coord_t z, const coord_t line_width) const override;
};

} // namespace cura

#endif