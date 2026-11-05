// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "types/Coord_t.h"
#include "types/geometry/Shape.h"

namespace curaengine
{
struct FakeRoofArea
{
    FakeRoofArea(types::geometry::Shape area, types::coord_t line_distance, bool fractional)
        : area_(area)
        , line_distance_(line_distance)
        , fractional_(fractional)
    {
    }
    /*!
     * \brief Area that should be a fake roof.
     */
    types::geometry::Shape area_;

    /*!
     * \brief Distance between support lines
     */
    types::coord_t line_distance_;

    /*!
     * \brief If the area should be added as a fractional support area.
     */
    bool fractional_;
};

} // namespace curaengine
