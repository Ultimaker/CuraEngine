// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include "ZigzagConnectorProcessor.h"

namespace curaengine::geometry
{
class OpenPolyline;
template<class T>
class LinesSet;
}

namespace curaengine::infill
{





/*!
 * This processor adds no connection. This is for line infill pattern.
 */
class NoZigZagConnectorProcessor : public ZigzagConnectorProcessor
{
public:
    NoZigZagConnectorProcessor(const ::curaengine::geometry::PointMatrix& rotation_matrix, ::curaengine::geometry::OpenLinesSet& result)
        : ZigzagConnectorProcessor(
              rotation_matrix,
              result,
              false,
              false, // settings for zig-zag end pieces, no use here
              false,
              0) // settings for skipping some zags, no use here
    {
    }

    void registerVertex(const ::curaengine::geometry::Point2LL& vertex);
    void registerScanlineSegmentIntersection(const ::curaengine::geometry::Point2LL& intersection, int scanline_index, ::curaengine::utils::coord_t min_distance_to_scanline);
    void registerPolyFinished();
};


} // namespace curaengine::infill
