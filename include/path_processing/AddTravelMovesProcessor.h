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

template<class OperationType, class ChildOperationType>
class AddTravelMovesProcessor : public InsertOperationsProcessor<OperationType, ChildOperationType>
{
public:
    explicit AddTravelMovesProcessor(const SpeedDerivatives& speed);

protected:
    std::shared_ptr<PrintOperation> makeOperation(const std::shared_ptr<ChildOperationType>& operation_before, const std::shared_ptr<ChildOperationType>& operation_after) override;

private:
    static std::optional<Point3LL> findStartPosition(const std::shared_ptr<ChildOperationType>& operation);

    static std::optional<Point3LL> findEndPosition(const std::shared_ptr<ChildOperationType>& operation);

private:
    const SpeedDerivatives& speed_;
    std::vector<std::shared_ptr<TravelMoveGenerator>> generators_;
};

} // namespace cura

#endif // PATHPROCESSING_ADDTRAVELMOVESPROCESSOR_H
