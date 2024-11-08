// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_ADDTRAVELMOVESPROCESSOR_H
#define PATHPROCESSING_ADDTRAVELMOVESPROCESSOR_H

#include <optional>

#include "path_planning/SpeedDerivatives.h"
#include "path_processing/InsertOperationsProcessor.h"

namespace cura
{

class PrintOperation;
class ExtruderPlan;
class TravelMoveGenerator;
class Point3LL;

class AddTravelMovesProcessor : public InsertOperationsProcessor
{
public:
    explicit AddTravelMovesProcessor(const SpeedDerivatives& speed);

protected:
    bool firstOperationMatches(const std::shared_ptr<PrintOperation>& operation) override;

    bool secondOperationMatches(const std::shared_ptr<PrintOperation>& first_operation, const std::shared_ptr<PrintOperation>& operation) override;

    std::shared_ptr<PrintOperation> makeOperation(const std::shared_ptr<PrintOperation>& operation_first, const std::shared_ptr<PrintOperation>& operation_second) override;

private:
    const SpeedDerivatives& speed_;
    std::vector<std::shared_ptr<TravelMoveGenerator>> generators_;
};

} // namespace cura

#endif // PATHPROCESSING_ADDTRAVELMOVESPROCESSOR_H
