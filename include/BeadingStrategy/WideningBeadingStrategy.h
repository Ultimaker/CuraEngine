//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef WIDENING_BEADING_STRATEGY_H
#define WIDENING_BEADING_STRATEGY_H

#include "BeadingStrategy.h"

namespace cura
{

/*!
 * This is a meta-strategy that can be applied on any other beading strategy. If
 * the part is thinner than a single line, this strategy adjusts the part so
 * that it becomes the minimum thickness of one line.
 *
 * This way, tiny pieces that are smaller than a single line will still be
 * printed.
 */
class WideningBeadingStrategy : public BeadingStrategy
{
public:
    /*!
     * Takes responsibility for deleting \param parent
     */
    WideningBeadingStrategy(BeadingStrategyPtr parent, const coord_t min_input_width, const coord_t min_output_width);

    virtual ~WideningBeadingStrategy() override = default;

    virtual Beading compute(coord_t thickness, coord_t bead_count) const override;
    virtual coord_t getOptimalThickness(coord_t bead_count) const override;
    virtual coord_t getTransitionThickness(coord_t lower_bead_count) const override;
    virtual coord_t getOptimalBeadCount(coord_t thickness) const override;
    virtual coord_t getTransitioningLength(coord_t lower_bead_count) const override;
    virtual float getTransitionAnchorPos(coord_t lower_bead_count) const override;
    virtual std::vector<coord_t> getNonlinearThicknesses(coord_t lower_bead_count) const override;
    virtual std::string toString() const override;

protected:
    BeadingStrategyPtr parent;
    const coord_t min_input_width;
    const coord_t min_output_width;
};

} // namespace cura
#endif // WIDENING_BEADING_STRATEGY_H
