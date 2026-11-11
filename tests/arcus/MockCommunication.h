// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MOCKCOMMUNICATION_H
#define MOCKCOMMUNICATION_H

#include <gmock/gmock.h>

#include "communication/Communication.h"
#include "types/Coord_t.h"
#include "types/LayerIndex.h"
#include "types/geometry/Polygon.h"
#include "types/geometry/Shape.h"

namespace curaengine
{

/*
 * No-op implementation of Communication.
 */
class MockCommunication : public communication::Communication
{
public:
    MOCK_CONST_METHOD0(hasSlice, bool());
    MOCK_CONST_METHOD1(sendProgress, void(double progress));
    MOCK_METHOD3(sendLayerComplete, void(const types::LayerIndex::value_type& layer_nr, const types::coord_t& z, const types::coord_t& thickness));
    MOCK_METHOD6(
        sendLineTo,
        void(
            const PrintFeatureType& type,
            const types::geometry::Point3LL& to,
            const types::coord_t& line_width,
            const types::coord_t& line_thickness,
            const types::Velocity& velocity,
            const PrintSegmentAttributes& print_attributes));
    MOCK_METHOD1(sendCurrentPosition, void(const types::geometry::Point3LL& position));
    MOCK_METHOD1(setExtruderForSend, void(const ExtruderTrain& extruder));
    MOCK_METHOD1(setLayerForSend, void(const types::LayerIndex::value_type& layer_nr));
    MOCK_METHOD0(sendOptimizedLayerData, void());
    MOCK_CONST_METHOD0(sendPrintTimeMaterialEstimates, void());
    MOCK_CONST_METHOD1(sendSliceUUID, void(const std::string& slice_uuid));
    MOCK_CONST_METHOD0(sendFinishedSlicing, void());
    MOCK_METHOD0(sliceNext, void());
    MOCK_CONST_METHOD2(sendPrintInformation, void(const std::vector<types::Duration>& time_estimates, const PrintInformation& print_information));
    MOCK_METHOD1(sendGCodePart, void(const std::string&));
};

} // namespace curaengine

#endif // MOCKCOMMUNICATION_H
