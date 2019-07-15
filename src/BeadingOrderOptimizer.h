//Copyright (c) 2019 Ultimaker B.V.


#ifndef BEADING_ORDER_OPTIMIZER_H
#define BEADING_ORDER_OPTIMIZER_H

#include "utils/polygon.h"
#include "utils/ExtrusionSegment.h"
#include "utils/ExtrusionJunction.h"

namespace arachne
{

/*!
 * Connecting ExtrusionSegments together into chains / polygons
 */
class BeadingOrderOptimizer
{
public:
    static void optimize(const std::vector<ExtrusionSegment>& segments, std::vector<std::vector<std::vector<ExtrusionJunction>>>& polygons_per_index, std::vector<std::vector<std::vector<ExtrusionJunction>>>& polylines_per_index);
};




} // namespace arachne
#endif // BEADING_ORDER_OPTIMIZER_H
