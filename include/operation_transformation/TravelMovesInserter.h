// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_ADDLAYERPLANTRAVELMOVESPROCESSOR_H
#define PATHPROCESSING_ADDLAYERPLANTRAVELMOVESPROCESSOR_H

#include <optional>

#include "DirectTravelMoveGenerator.h"
#include "ExtruderNumber.h"
#include "operation_transformation/PrintOperationTransformer.h"
#include "print_operation/PrintPlan.h"

namespace cura
{

class TravelMoveGenerator;
class TravelRoute;
struct GCodePathConfig;
struct SpeedDerivatives;

class TravelMovesInserter : public PrintOperationTransformer<PrintPlan>
{
public:
    explicit TravelMovesInserter();

    void process(PrintPlan* layer_plan) override;

private:
    void appendTravelsMovesBetweenChildren(const std::shared_ptr<PrintOperationSequence>& sequence, const SpeedDerivatives& speed);

    void appendTravelMovesRecursively(const std::shared_ptr<PrintOperation>& operation, const SpeedDerivatives& speed);

    const std::shared_ptr<TravelRoute> makeTravelRoute(const Point3LL& start_position, const Point3LL& end_position, const SpeedDerivatives& speed);

    const SpeedDerivatives getTravelSpeed(
        const std::map<ExtruderNumber, SpeedDerivatives>& extruders_travel_speed_initial,
        const std::map<ExtruderNumber, SpeedDerivatives>& extruders_travel_speed_up,
        const size_t initial_speedup_layer_count,
        const LayerIndex& layer_nr,
        const ExtruderNumber extruder_nr) const;

    std::map<ExtruderNumber, SpeedDerivatives> makeTravelSpeedsInitial(const std::vector<ExtruderNumber>& used_extruders) const;

    std::map<ExtruderNumber, SpeedDerivatives> makeTravelSpeedsUp(const std::vector<ExtruderNumber>& used_extruders) const;

    std::map<ExtruderNumber, SpeedDerivatives> makeTravelSpeeds(
        const std::vector<ExtruderNumber>& used_extruders,
        const std::string& setting_speed,
        const std::string& setting_acceleration,
        const std::string& setting_jerk) const;

private:
    std::vector<std::shared_ptr<TravelMoveGenerator>> generators_;
    std::shared_ptr<DirectTravelMoveGenerator> direct_travel_move_generator_;
};

} // namespace cura

#endif // PATHPROCESSING_ADDLAYERPLANTRAVELMOVESPROCESSOR_H
