// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "geometry/Shape.h"
#include "utils/Coord_t.h"

namespace curaengine
{
struct FakeRoofArea
{
    FakeRoofArea(Shape area, coord_t line_distance, bool fractional)
        : area_(area)
        , line_distance_(line_distance)
        , fractional_(fractional)
    {
    }
    /*!
     * \brief Area that should be a fake roof.
     */
    Shape area_;

    /*!
     * \brief Distance between support lines
     */
    coord_t line_distance_;

    /*!
     * \brief If the area should be added as a fractional support area.
     */
    bool fractional_;
};

} // namespace curaengine
