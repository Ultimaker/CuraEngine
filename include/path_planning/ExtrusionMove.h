// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_EXTRUSIONMOVE_H
#define PATHPLANNING_EXTRUSIONMOVE_H

#include "path_planning/ExtruderMove.h"

namespace cura
{

class ExtrusionMove : public ExtruderMove
{
public:
    explicit ExtrusionMove(const Point3LL& position);

    void write(PathExporter& gcode, const LayerPlan& layer_plan, const ExtruderMoveSet& extruder_move_set) const override;
};

} // namespace cura

#endif // PATHPLANNING_EXTRUSIONMOVE_H
