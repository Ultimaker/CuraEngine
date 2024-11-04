// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_EXTRUDERMOVESET_H
#define PATHPLANNING_EXTRUDERMOVESET_H

#include <memory>
#include <vector>

#include "GCodePathConfig.h"
#include "SpaceFillType.h"
#include "path_planning/TimeMaterialEstimates.h"
#include "settings/types/Ratio.h"
#include "utils/Coord_t.h"

namespace cura
{

class ExtruderMove;
class SliceMeshStorage;
class PathExporter;
class LayerPlan;

class ExtruderMoveSet
{
public:
    virtual ~ExtruderMoveSet() = default; // Force class being polymorphic

    bool empty() const noexcept;

    /*!
     * Write the planned paths to gcode
     *
     * \param gcode The gcode to write the planned paths to
     */
    void write(PathExporter& exporter, const LayerPlan& layer_plan) const;

    coord_t getZOffset() const;

    const Ratio& getSpeedFactor() const;

    const Ratio& getSpeedBackPressureFactor() const;

protected:
    void appendExtruderMove(const std::shared_ptr<ExtruderMove>& extruder_move);

private:
    std::vector<std::shared_ptr<ExtruderMove>> extruder_moves_; //<! The list of moves to be processed for this path
    coord_t z_offset_{ 0 }; //<! Vertical offset from 'full' layer height, applied to the whole path (can be different from the one in the config)
    std::shared_ptr<const SliceMeshStorage> mesh_; //!< Which mesh this path belongs to, if any. If it's not part of any mesh, the mesh should be nullptr;
    SpaceFillType space_fill_type_{ SpaceFillType::None }; //!< The type of space filling of which this path is a part
    Ratio speed_factor_{ 1.0 }; //!< A speed factor that is multiplied with the travel speed. This factor can be used to change the travel speed.
    Ratio speed_back_pressure_factor_{ 1.0 }; // <! The factor the (non-travel) speed should be multiplied with as a consequence of back pressure compensation.
    double fan_speed_{ GCodePathConfig::FAN_SPEED_DEFAULT }; //!< fan speed override for this path, value should be within range 0-100 (inclusive) and ignored otherwise
};

} // namespace cura

#endif // PATHPLANNING_EXTRUDERMOVESET_H
