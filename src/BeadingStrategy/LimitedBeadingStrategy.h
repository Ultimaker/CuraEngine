//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef LIMITED_BEADING_STRATEGY_H
#define LIMITED_BEADING_STRATEGY_H

#include "BeadingStrategy.h"
#include "../utils/logoutput.h"
#include "../utils/macros.h"

namespace cura
{

/*!
 * This is a meta-strategy that can be applied on top of any other beading
 * strategy, which limits the thickness of the walls to the thickness that the
 * lines can reasonably print.
 *
 * The width of the wall is limited to the maximum number of contours times the
 * maximum width of each of these contours.
 *
 * If the width of the wall gets limited, this strategy outputs one additional
 * bead with 0 width. This bead is used to denote the limits of the walled area.
 * Other structures can then use this border to align their structures to, such
 * as to create correctly overlapping infill or skin, or to align the infill
 * pattern to any extra infill walls.
 */
class LimitedBeadingStrategy : public BeadingStrategy
{
public:
    const coord_t max_bead_count;
    const BeadingStrategy* parent;
    LimitedBeadingStrategy(const coord_t max_bead_count, BeadingStrategy* parent)
    : BeadingStrategy(parent->getOptimalWidth(), /*default_transition_length=*/-1, parent->getTransitioningAngle())
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
    coord_t getOptimalThickness(coord_t bead_count) const override;
    coord_t getTransitionThickness(coord_t lower_bead_count) const override;
    coord_t getOptimalBeadCount(coord_t thickness) const override;
    virtual std::string toString() const override { return std::string("LimitedBeadingStrategy+") + parent->toString();}
    
    coord_t getTransitioningLength(coord_t lower_bead_count) const override
    {
        return parent->getTransitioningLength(lower_bead_count);
    }
    
    float getTransitionAnchorPos(coord_t lower_bead_count) const override
    {
        return parent->getTransitionAnchorPos(lower_bead_count);
    }
};


} // namespace cura
#endif // LIMITED_DISTRIBUTED_BEADING_STRATEGY_H
