// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_TRAVELMOVE_H
#define PATHPLANNING_TRAVELMOVE_H

#include "print_operation/ExtruderMove.h"

namespace cura
{

class TravelMove : public ExtruderMove
{
public:
    explicit TravelMove(const Point3LL& position);

    void write(PlanExporter& exporter) const override;
};

} // namespace cura

#endif // PATHPLANNING_TRAVELMOVE_H
