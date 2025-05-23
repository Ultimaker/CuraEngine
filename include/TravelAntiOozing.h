// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef TRAVELANTIOOZING_H
#define TRAVELANTIOOZING_H

#include "geometry/Point2LL.h"
#include "settings/types/Ratio.h"

namespace cura
{

struct ZHopAntiOozing
{
    double amount{ 0 }; //!< The absolute amount of retracted material (in mm) to be reached while the nozzle is moved up/down
    Ratio ratio; //!< The ratio of the full z-hop move that contains a retract/prime
};

struct TravelAntiOozing
{
    double amount_while_still{ 0 }; //!< The absolute amount of retracted material (in mm) to be reached while the nozzle is still
    ZHopAntiOozing z_hop; //!< The amount and ratio of retracted material to be processed during z-hop move
    double amount_while_travel{ 0 }; //!< The absolute amount of retracted material (in mm) to be reached while the nozzle is traveling
    Point2LL segment_split_position; //!< The intermediate position on the last/first segment that contains a retract/prime, where it should actually stop/start
    std::vector<double> amount_by_segment; //!< For each intermediate segment containing a retraction/prime, this is the absolute amount to be reached at the end of the segment
};

} // namespace cura

#endif /* TRAVELANTIOOZING_H */
