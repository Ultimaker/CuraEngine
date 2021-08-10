//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#ifndef CENTER_DEVIATION_BEADING_STRATEGY_H
#define CENTER_DEVIATION_BEADING_STRATEGY_H

#include "../settings/types/Ratio.h" //For the wall transition threshold.
#include "BeadingStrategy.h"
#ifdef BUILD_TESTS
    #include <gtest/gtest_prod.h> //Friend tests, so that they can inspect the privates.
#endif

namespace cura
{

/*!
 * This beading strategy makes the deviation in the thickness of the part
 * entirely compensated by the innermost wall.
 *
 * The outermost walls all use the ideal width, as far as possible.
 */
class CenterDeviationBeadingStrategy : public BeadingStrategy
{
#ifdef BUILD_TESTS
    FRIEND_TEST(CenterDeviationBeadingStrategy, Construction);
#endif

private:
    /*!
     * Minimum allowed line width.
     *
     * If the innermost two lines would be below this line width, it should use
     * a single line instead. If the innermost center line would be below this
     * line width, it should be left out as a gap.
     */
    coord_t minimum_line_width;
public:
    CenterDeviationBeadingStrategy(const coord_t pref_bead_width, const AngleRadians transitioning_angle, const Ratio wall_transition_threshold);

    virtual ~CenterDeviationBeadingStrategy() override {}
    Beading compute(coord_t thickness, coord_t bead_count) const override;
    coord_t getOptimalThickness(coord_t bead_count) const override;
    coord_t getTransitionThickness(coord_t lower_bead_count) const override;
    coord_t getOptimalBeadCount(coord_t thickness) const override;
};

} // namespace cura
#endif // CENTER_DEVIATION_BEADING_STRATEGY_H
