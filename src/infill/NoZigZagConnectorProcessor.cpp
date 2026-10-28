// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/NoZigZagConnectorProcessor.h"

#include "utils/macros.h"


using namespace curaengine::geometry;
using namespace curaengine::utils;

namespace curaengine::infill
{

void NoZigZagConnectorProcessor::registerVertex(const Point2LL&)
{
    // No need to add anything.
}

void NoZigZagConnectorProcessor::registerScanlineSegmentIntersection(const Point2LL&, int, coord_t)
{
    // No need to add anything.
}

void NoZigZagConnectorProcessor::registerPolyFinished()
{
    // No need to add anything.
}

} // namespace curaengine::infill
