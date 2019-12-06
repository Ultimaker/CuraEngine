//Copyright (c) 2019 Ultimaker B.V.


#ifndef LIMITED_BEADING_STRATEGY_H
#define LIMITED_BEADING_STRATEGY_H

#include "BeadingStrategy.h"
#include "utils/logoutput.h"
#include "utils/macros.h"

namespace arachne
{

/*!
 * Beading strategy which evenly subdivides the thickness and tries to stay close to the optimal width.
 */
class LimitedBeadingStrategy : public BeadingStrategy
{
public:
    const coord_t max_bead_count;
    const BeadingStrategy* parent;
    LimitedBeadingStrategy(const coord_t max_bead_count, BeadingStrategy* parent)
    : BeadingStrategy(parent->optimal_width, parent->transitioning_angle)
    , max_bead_count(max_bead_count)
    , parent(parent)
    {
        if (max_bead_count % 2 == 1)
        {
            RUN_ONCE(logWarning("LimitedBeadingStrategy with odd bead count is odd indeed!\n"));
        }
    }
    virtual ~LimitedBeadingStrategy() override
    {
        delete parent;
    }
    Beading compute(coord_t thickness, coord_t bead_count) const override;
    coord_t optimal_thickness(coord_t bead_count) const override;
    coord_t transition_thickness(coord_t lower_bead_count) const override;
    coord_t optimal_bead_count(coord_t thickness) const override;
    std::vector<coord_t> getNonlinearThicknesses(coord_t lower_bead_count) const override;
    virtual std::string toString() const override { return std::string("LimitedBeadingStrategy+") + parent->toString();}
};




} // namespace arachne
#endif // LIMITED_DISTRIBUTED_BEADING_STRATEGY_H
