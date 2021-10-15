// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.


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
    // For uneven numbers of lines: Minimum line width for which the middle line will be split into two lines.
    coord_t minimum_line_width_split;

    // For even numbers of lines: Minimum line width for which a new middle line will be added between the two innermost lines.
    coord_t minimum_line_width_add;

  public:
    CenterDeviationBeadingStrategy(coord_t pref_bead_width,
                                   AngleRadians transitioning_angle,
                                   Ratio wall_split_middle_threshold,
                                   Ratio wall_add_middle_threshold);

    ~CenterDeviationBeadingStrategy() override{};
    Beading compute(coord_t thickness, coord_t bead_count) const override;
    coord_t getOptimalThickness(coord_t bead_count) const override;
    coord_t getTransitionThickness(coord_t lower_bead_count) const override;
    coord_t getOptimalBeadCount(coord_t thickness) const override;
};

} // namespace cura
#endif // CENTER_DEVIATION_BEADING_STRATEGY_H
