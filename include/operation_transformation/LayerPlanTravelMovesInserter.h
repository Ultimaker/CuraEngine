// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_ADDLAYERPLANTRAVELMOVESPROCESSOR_H
#define PATHPROCESSING_ADDLAYERPLANTRAVELMOVESPROCESSOR_H

#include <optional>

#include "operation_transformation/PrintOperationTransformer.h"
#include "print_operation/LayerPlan.h"

namespace cura
{

class TravelMoveGenerator;

class LayerPlanTravelMovesInserter : public PrintOperationTransformer<LayerPlan>
{
public:
    explicit LayerPlanTravelMovesInserter();

    void process(LayerPlan* layer_plan) override;

private:
    void appendTravelsMovesBetweenChildren(const std::shared_ptr<PrintOperationSequence>& sequence, const SpeedDerivatives& speed);

    void appendTravelMovesRecursively(const std::shared_ptr<PrintOperation>& operation, const SpeedDerivatives& speed);

    const std::shared_ptr<PrintOperation> makeTravelMove(const Point3LL& start_position, const Point3LL& end_position, const SpeedDerivatives& speed);

private:
    std::vector<std::shared_ptr<TravelMoveGenerator>> generators_;
};

} // namespace cura

#endif // PATHPROCESSING_ADDLAYERPLANTRAVELMOVESPROCESSOR_H
