// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "seam/seam_corners.h"
#include "types/geometry/Point2LL.h"

namespace curaengine::types::scoring
{

/*!
 * Gets the position of an element that is processed by the position-based criteria. Add an overload for each type of element that has to
 * be supported
 */
inline const geometry::Point2LL& getElementPosition(const geometry::Point2LL& position)
{
    return position;
}

inline const geometry::Point2LL& getElementPosition(const seam::SeamCandidate& candidate)
{
    return candidate.position;
}

} // namespace curaengine::types::scoring
