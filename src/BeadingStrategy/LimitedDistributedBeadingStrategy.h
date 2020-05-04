//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef LIMITED_DISTRIBUTED_BEADING_STRATEGY_H
#define LIMITED_DISTRIBUTED_BEADING_STRATEGY_H

#include "DistributedBeadingStrategy.h"
#include "../utils/logoutput.h"
#include "../utils/macros.h"

namespace arachne
{
    using namespace cura;

/*!
 * This is a meta-strategy that can be applied on top of any other beading
 * strategy, which limits the thickness of the walls to the summed ideal width
 * of the wall lines.
 *
 * The wall thickness can never be such that the maximum number of lines is used
 * and the thickness of these lines is greater than the ideal width. There will
 * always be a thinner-than-ideal line or a line less than the maximum, unless
 * the maximum thickness is reached; then all lines are ideal width.
 */
class LimitedDistributedBeadingStrategy : public DistributedBeadingStrategy
{
public:
    const coord_t max_bead_count;
    LimitedDistributedBeadingStrategy(const coord_t optimal_width, coord_t default_transition_length, const coord_t max_bead_count, float transitioning_angle)
    : DistributedBeadingStrategy(optimal_width, default_transition_length, transitioning_angle)
    , max_bead_count(max_bead_count)
    {
        if (max_bead_count % 2 == 1)
        {
            RUN_ONCE(logWarning("LimitedDistributedBeadingStrategy with odd bead count is odd indeed!\n"));
        }
        name = "LimitedDistributedBeadingStrategy";
    }
    virtual ~LimitedDistributedBeadingStrategy() override
    {}
    Beading compute(coord_t thickness, coord_t bead_count) const override;
    coord_t getOptimalThickness(coord_t bead_count) const override;
    coord_t getTransitionThickness(coord_t lower_bead_count) const override;
    coord_t getOptimalBeadCount(coord_t thickness) const override;
};

} // namespace arachne
#endif // LIMITED_DISTRIBUTED_BEADING_STRATEGY_H
