// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_FEATUREEXTRUSIONSORDEROPTIMIZER_H
#define PATHPROCESSING_FEATUREEXTRUSIONSORDEROPTIMIZER_H

#include "path_planning/ExtruderPlan.h"
#include "path_processing/PrintOperationProcessor.h"

namespace cura
{

class FeatureExtrusionsOrderOptimizer final : public PrintOperationProcessor<ExtruderPlan>
{
public:
    explicit FeatureExtrusionsOrderOptimizer(const Point3LL& previous_position);

    void process(ExtruderPlan* extruder_plan) override;


private:
    void optimizeExtruderSequencesOrder(const std::shared_ptr<FeatureExtrusion>& feature, Point3LL& current_position);

private:
    const Point3LL previous_position_;
};

} // namespace cura

#endif // PATHPROCESSING_FEATUREEXTRUSIONSORDEROPTIMIZER_H
