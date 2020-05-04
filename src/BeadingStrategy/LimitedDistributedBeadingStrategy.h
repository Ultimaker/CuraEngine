//Copyright (c) 2019 Ultimaker B.V.


#ifndef LIMITED_DISTRIBUTED_BEADING_STRATEGY_H
#define LIMITED_DISTRIBUTED_BEADING_STRATEGY_H

#include "DistributedBeadingStrategy.h"
#include "../utils/logoutput.h"
#include "../utils/macros.h"

namespace arachne
{
    using namespace cura;

/*!
 * Beading strategy which evenly subdivides the thickness and tries to stay close to the optimal width.
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
