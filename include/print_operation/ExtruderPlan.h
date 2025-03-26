// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "print_operation/ExtruderPlanPtr.h"
#include "print_operation/PrintOperationSequence.h"

namespace cura
{

class FeatureExtrusion;
struct SpeedDerivatives;

class ExtruderPlan : public PrintOperationSequence
{
public:
    explicit ExtruderPlan(const size_t extruder_nr) noexcept;

    size_t getExtruderNr() const noexcept;

    void appendFeatureExtrusion(const std::shared_ptr<FeatureExtrusion>& feature_extrusion, const bool check_non_empty = true);

    static ExtruderPlanPtr find(const std::vector<ExtruderPlanPtr>& extruder_plans, const size_t extruder_nr);

private:
    const size_t extruder_nr_;
};

} // namespace cura
