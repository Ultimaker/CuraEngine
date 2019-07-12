//Copyright (c) 2019 Ultimaker B.V.


#ifndef LIMITED_DISTRIBUTED_BEADING_STRATEGY_H
#define LIMITED_DISTRIBUTED_BEADING_STRATEGY_H

#include "DistributedBeadingStrategy.h"
#include "utils/logoutput.h"
#include "utils/macros.h"

namespace arachne
{

/*!
 * Beading strategy which evenly subdivides the thickness and tries to stay close to the optimal width.
 */
class LimitedDistributedBeadingStrategy : public DistributedBeadingStrategy
{
public:
    const coord_t max_bead_count;
    LimitedDistributedBeadingStrategy(const coord_t min_width, const coord_t optimal_width, const coord_t max_width, const coord_t max_bead_count, float transitioning_angle)
    : DistributedBeadingStrategy(min_width, optimal_width, max_width, transitioning_angle)
    , max_bead_count(max_bead_count)
    {
        if (max_bead_count % 2 == 1)
        {
            RUN_ONCE(logWarning("LimitedDistributedBeadingStrategy with odd bead count is odd indeed!\n"));
        }
    }
    Beading compute(coord_t thickness, coord_t bead_count) const override;
    coord_t optimal_thickness(coord_t bead_count) const override;
    coord_t transition_thickness(coord_t lower_bead_count) const override;
    coord_t optimal_bead_count(coord_t thickness) const override;
};




} // namespace arachne
#endif // LIMITED_DISTRIBUTED_BEADING_STRATEGY_H
