// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_EXTRUDERMOVE_H
#define PATHPLANNING_EXTRUDERMOVE_H

#include "geometry/Point3LL.h"
#include "path_planning/PrintOperation.h"

namespace cura
{

class PathExporter;
class LayerPlan;
class ContinuousExtruderMoveSequence;

class ExtruderMove : public PrintOperation
{
public:
    explicit ExtruderMove(const Point3LL& position);

    const Point3LL& getPosition() const;

    void setPosition(const Point3LL& position);

    std::optional<Point3LL> findEndPosition() const override;

private:
    /*!
     * The target position to move the extruder to. The X and Y coordinates are absolute positions, and the Z coordinate
     * is an offset to be applied for this specific position
     */
    Point3LL position_;
};

} // namespace cura

#endif // PATHPLANNING_EXTRUDERMOVE_H
