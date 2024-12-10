// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "geometry/Point3LL.h"
#include "print_operation/PrintOperationSequence.h"

namespace cura
{

class ExtruderMove;
class SliceMeshStorage;
class PlanExporter;
class LayerPlan;
class Point3LL;

class ContinuousExtruderMoveSequence : public PrintOperationSequence
{
public:
    explicit ContinuousExtruderMoveSequence(bool closed, const Point3LL& start_position = Point3LL());

    void appendExtruderMove(const std::shared_ptr<ExtruderMove>& extruder_move);

    std::optional<Point3LL> findStartPosition() const override;

    bool isClosed() const;

    void reorderToEndWith(const std::shared_ptr<ExtruderMove>& extruder_move);

    void reverse();

private:
    Point3LL start_position_;
    const bool closed_;
};

} // namespace cura
