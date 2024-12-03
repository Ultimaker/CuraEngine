// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "ExtruderPlanPtr.h"
#include "print_operation/PrintOperationSequence.h"
#include "settings/types/LayerIndex.h"

namespace cura
{

struct PathConfigStorage;

class LayerPlan : public PrintOperationSequence
{
public:
    LayerPlan(const LayerIndex& layer_index, const coord_t z, const coord_t thickness, const std::shared_ptr<PathConfigStorage>& configs);

    virtual ~LayerPlan() = default;

    void appendExtruderPlan(const ExtruderPlanPtr& extruder_plan, const bool check_non_empty = true);

    LayerIndex getLayerIndex() const;

    coord_t getZ() const;

    coord_t getThickness() const;

    void write(PlanExporter& exporter) const override;

    std::optional<Point3LL> findExtruderStartPosition() const;

    Point3LL getAbsolutePosition(const ContinuousExtruderMoveSequence& move_sequence, const Point3LL& relative_position) const;

private:
    const LayerIndex layer_index_;
    const coord_t z_;
    const coord_t thickness_;
    const std::shared_ptr<PathConfigStorage> configs_;
};

} // namespace cura
