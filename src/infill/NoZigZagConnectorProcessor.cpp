// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/NoZigZagConnectorProcessor.h"

#include "utils/macros.h"


namespace curaengine::infill
{

void NoZigZagConnectorProcessor::registerVertex(const ::curaengine::geometry::Point2LL&)
{
    // No need to add anything.
}

void NoZigZagConnectorProcessor::registerScanlineSegmentIntersection(const ::curaengine::geometry::Point2LL&, int, ::curaengine::utils::coord_t)
{
    // No need to add anything.
}

void NoZigZagConnectorProcessor::registerPolyFinished()
{
    // No need to add anything.
}

} // namespace curaengine::infill
