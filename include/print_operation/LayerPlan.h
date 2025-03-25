// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "ExtruderNumber.h"
#include "ExtruderPlanPtr.h"
#include "print_operation/PrintOperationSequence.h"
#include "settings/types/LayerIndex.h"

namespace cura
{

struct PathConfigStorage;
class ExtruderChange;

class LayerPlan : public PrintOperationSequence
{
public:
    LayerPlan(const LayerIndex& layer_index, const coord_t z, const coord_t thickness, const std::shared_ptr<PathConfigStorage>& configs);

    virtual ~LayerPlan() = default;

    void appendExtruderPlan(const ExtruderPlanPtr& extruder_plan, const bool check_non_empty = true);

    void insertExtruderChangeAfter(const ExtruderPlanPtr& extruder_plan, const std::shared_ptr<ExtruderChange>& extruder_change);

    LayerIndex getLayerIndex() const;

    coord_t getZ() const;

    coord_t getThickness() const;

    const std::shared_ptr<PathConfigStorage>& getConfigsStorage() const;

    void write(PlanExporter& exporter) const override;

    Point3LL getAbsolutePosition(const Point3LL& relative_position) const;

    ExtruderPlanPtr findFirstExtruderPlan(const ExtruderNumber& extruder_nr) const;

private:
    const LayerIndex layer_index_;
    const coord_t z_;
    const coord_t thickness_;
    const std::shared_ptr<PathConfigStorage> configs_;
};

} // namespace cura
