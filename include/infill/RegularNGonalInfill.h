// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_REGULARNGONALINFILL_H
#define INFILL_REGULARNGONALINFILL_H

#include "infill/AbstractLinesInfill.h"

namespace cura
{

class RegularNGonalInfill : public AbstractLinesInfill
{
public:
    RegularNGonalInfill() = default;

    ~RegularNGonalInfill() override = default;

protected:
    OpenLinesSet generateParallelLines(const coord_t line_distance, const Shape& in_outline, const coord_t z, const coord_t line_width) const override;
};

} // namespace cura

#endif