// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "infill/NoZigZagConnectorProcessor.h"

#include "utils/macros.h"


namespace curaengine::infill
{
using namespace types::geometry;
using namespace types;
using namespace utils;

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
