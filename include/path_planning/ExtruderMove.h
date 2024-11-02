// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_EXTRUDERMOVE_H
#define PATHPLANNING_EXTRUDERMOVE_H

#include "geometry/Point3LL.h"

namespace cura
{

class PathExporter;
class LayerPlan;
class ExtruderMoveSet;

class ExtruderMove
{
public:
    explicit ExtruderMove(const Point3LL& position);

    /*!
     * Write the planned paths to gcode
     *
     * \param gcode The gcode to write the planned paths to
     */
    virtual void write(PathExporter& exporter, const LayerPlan& layer_plan, const ExtruderMoveSet& extruder_move_set) const = 0;

protected:
    Point3LL getAbsolutePosition(const LayerPlan& layer_plan, const ExtruderMoveSet& extruder_move_set) const;

    const Point3LL& getPosition() const;

private:
    /*!
     * The target position to move the extruder to. The X and Y coordinates are absolute positions, and the Z coordinate
     * is an offset to be applied for this specific position
     */
    Point3LL position_;
};

} // namespace cura

#endif // PATHPLANNING_EXTRUDERMOVE_H
