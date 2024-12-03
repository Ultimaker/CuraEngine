// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "print_operation/PrintOperationSequence.h"

namespace cura
{

class FeatureExtrusion;
struct SpeedDerivatives;

class ExtruderPlan : public PrintOperationSequence
{
public:
    explicit ExtruderPlan(const size_t extruder_nr, const SpeedDerivatives& travel_speed) noexcept;

    size_t getExtruderNr() const noexcept;

    const SpeedDerivatives& getTravelSpeed() const noexcept;

    void appendFeatureExtrusion(const std::shared_ptr<FeatureExtrusion>& feature_extrusion, const bool check_non_empty = true);

private:
    const size_t extruder_nr_;
#warning use a shared_ptr
    const SpeedDerivatives& travel_speed_;
};

} // namespace cura
