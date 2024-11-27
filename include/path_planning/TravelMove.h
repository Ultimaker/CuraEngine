// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPLANNING_TRAVELMOVE_H
#define PATHPLANNING_TRAVELMOVE_H

#include "path_planning/ExtruderMove.h"

namespace cura
{

class TravelMove : public ExtruderMove
{
public:
    explicit TravelMove(const Point3LL& position);

    virtual void write(PathExporter& exporter) const override;
};

} // namespace cura

#endif // PATHPLANNING_TRAVELMOVE_H
