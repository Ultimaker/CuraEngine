// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_ADDTRAVELMOVESPROCESSOR_H
#define PATHPROCESSING_ADDTRAVELMOVESPROCESSOR_H

#include "path_planning/FeatureExtrusion.h"
#include "path_planning/SpeedDerivatives.h"
#include "path_processing/InsertOperationsProcessor.h"

namespace cura
{

class PrintOperation;
class ExtruderPlan;
class Settings;
class TravelMoveGenerator;

class AddTravelMovesProcessor : public InsertOperationsProcessor<ExtruderPlan, FeatureExtrusion>
{
public:
    explicit AddTravelMovesProcessor(const SpeedDerivatives& speed);

protected:
    std::shared_ptr<PrintOperation> makeOperation(const std::shared_ptr<FeatureExtrusion>& operation_before, const std::shared_ptr<FeatureExtrusion>& operation_after) override;

private:
    const SpeedDerivatives& speed_;
    std::vector<std::shared_ptr<TravelMoveGenerator>> generators_;
};

} // namespace cura

#endif // PATHPROCESSING_ADDTRAVELMOVESPROCESSOR_H
