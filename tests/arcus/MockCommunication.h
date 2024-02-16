// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MOCKCOMMUNICATION_H
#define MOCKCOMMUNICATION_H

#include <gmock/gmock.h>

#include "communication/Communication.h" //The interface we're implementing.
#include "settings/types/LayerIndex.h"
#include "utils/Coord_t.h"
#include "utils/polygon.h" //In the signature of Communication.

namespace cura
{

/*
 * No-op implementation of Communication.
 */
class MockCommunication : public Communication
{
public:
    MOCK_CONST_METHOD0(hasSlice, bool());
    MOCK_CONST_METHOD0(isSequential, bool());
    MOCK_CONST_METHOD1(sendProgress, void(double progress));
    MOCK_METHOD3(sendLayerComplete, void(const LayerIndex::value_type& layer_nr, const coord_t& z, const coord_t& thickness));
    MOCK_METHOD5(sendPolygons, void(const PrintFeatureType& type, const Polygons& polygons, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity));
    MOCK_METHOD5(
        sendPolygon,
        void(const PrintFeatureType& type, const ConstPolygonRef& polygon, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity));
    MOCK_METHOD5(sendLineTo, void(const PrintFeatureType& type, const Point2LL& to, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity));
    MOCK_METHOD1(sendCurrentPosition, void(const Point2LL& position));
    MOCK_METHOD1(setExtruderForSend, void(const ExtruderTrain& extruder));
    MOCK_METHOD1(setLayerForSend, void(const LayerIndex::value_type& layer_nr));
    MOCK_METHOD0(sendOptimizedLayerData, void());
    MOCK_CONST_METHOD0(sendPrintTimeMaterialEstimates, void());
    MOCK_METHOD0(beginGCode, void());
    MOCK_METHOD0(flushGCode, void());
    MOCK_CONST_METHOD1(sendGCodePrefix, void(const std::string& prefix));
    MOCK_CONST_METHOD1(sendSliceUUID, void(const std::string& slice_uuid));
    MOCK_CONST_METHOD0(sendFinishedSlicing, void());
    MOCK_METHOD0(sliceNext, void());
};

} // namespace cura

#endif // MOCKCOMMUNICATION_H
