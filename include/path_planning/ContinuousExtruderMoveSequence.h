// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_CONTINUOUSEXTRUDERMOVESEQUENCE_H
#define PATHPLANNING_CONTINUOUSEXTRUDERMOVESEQUENCE_H

#include "GCodePathConfig.h"
#include "SpaceFillType.h"
#include "geometry/Point3LL.h"
#include "path_planning/PrintOperationSequence.h"
#include "path_planning/TimeMaterialEstimates.h"
#include "settings/types/Ratio.h"
#include "utils/Coord_t.h"

namespace cura
{

class ExtruderMove;
class SliceMeshStorage;
class PathExporter;
class LayerPlan;
class Point3LL;

class ContinuousExtruderMoveSequence : public PrintOperationSequence
{
public:
    explicit ContinuousExtruderMoveSequence(bool closed, const Point3LL& start_position = Point3LL());

    void appendExtruderMove(const Point3LL& position, const Ratio& line_width_ratio = 1.0_r);

    std::optional<Point3LL> findStartPosition() const override;

    coord_t getZOffset() const;

    const Ratio& getSpeedFactor() const;

    const Ratio& getSpeedBackPressureFactor() const;

    bool isClosed() const;

    void reorderToEndWith(const std::shared_ptr<ExtruderMove>& extruder_move);

    void reverse();

private:
    Point3LL start_position_;
    bool closed_;
    coord_t z_offset_{ 0 }; //<! Vertical offset from 'full' layer height, applied to the whole path (can be different from the one in the config)
    std::shared_ptr<const SliceMeshStorage> mesh_; //!< Which mesh this path belongs to, if any. If it's not part of any mesh, the mesh should be nullptr;
    SpaceFillType space_fill_type_{ SpaceFillType::None }; //!< The type of space filling of which this path is a part
    Ratio speed_factor_{ 1.0 }; //!< A speed factor that is multiplied with the travel speed. This factor can be used to change the travel speed.
    Ratio speed_back_pressure_factor_{ 1.0 }; // <! The factor the (non-travel) speed should be multiplied with as a consequence of back pressure compensation.
    double fan_speed_{ GCodePathConfig::FAN_SPEED_DEFAULT }; //!< fan speed override for this path, value should be within range 0-100 (inclusive) and ignored otherwise
};

} // namespace cura

#endif // PATHPLANNING_CONTINUOUSEXTRUDERMOVESEQUENCE_H
