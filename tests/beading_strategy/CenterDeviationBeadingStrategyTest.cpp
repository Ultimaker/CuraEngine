//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include "../../src/BeadingStrategy/CenterDeviationBeadingStrategy.h" //Code under test.

namespace cura
{

/*!
 * Tests calling the constructor of the CenterDeviationBeadingStrategy and
 * whether it sets members correctly.
 */
TEST(CenterDeviationBeadingStrategy, Construction)
{
    CenterDeviationBeadingStrategy strategy(400, 0.6, 0.5); //Pretty standard settings.
    EXPECT_EQ(strategy.minimum_line_width, 200) << "Since the transition threshold was 50%, the minimum line width should be 0.5 times the preferred width.";
    strategy = CenterDeviationBeadingStrategy(400, 0.6, 0);
    EXPECT_EQ(strategy.minimum_line_width, 0) << "Since the transition threshold was 0%, the minimum line width should be 0.";
    strategy = CenterDeviationBeadingStrategy(0, 0.6, 0.5);
    EXPECT_EQ(strategy.minimum_line_width, 0) << "Since the line width was 0%, the minimum line width should also be 0.";
    strategy = CenterDeviationBeadingStrategy(400, 0.6, 1);
    EXPECT_EQ(strategy.minimum_line_width, 400) << "Since the transition threshold was 100%, the minimum line width equals the preferred width.";
}

/*!
 * Tests getting the optimal thickness with Center Deviation.
 *
 * This should simply be a multiplication of the line width, when using Center
 * Deviation. It doesn't adjust the line widths by itself.
 */
TEST(CenterDeviationBeadingStrategy, GetOptimalThickness)
{
    constexpr coord_t line_width = 400;
    CenterDeviationBeadingStrategy strategy(line_width, 0.6, 0.5);
    EXPECT_EQ(strategy.getOptimalThickness(0), 0) << "With 0 beads, you'll fill 0 space.";
    EXPECT_EQ(strategy.getOptimalThickness(1), line_width) << "With 1 bead, optimally you'll want to print that bead at the optimal line width.";
    EXPECT_EQ(strategy.getOptimalThickness(4), 4 * line_width) << "With 4 beads, optimally fill 4 line widths.";
}

/*!
 * Test getting the width at which we need to transition to a greater number of
 * lines.
 */
TEST(CenterDeviationBeadingStrategy, GetTransitionThickness)
{
    constexpr coord_t line_width = 400;

    //Transition ratio 25%.
    CenterDeviationBeadingStrategy strategy(line_width, 0.6, 0.25);
    EXPECT_EQ(strategy.getTransitionThickness(0), 0.25 * line_width) << "The transition from 0 beads to 1 happens at 25% line width.";
    EXPECT_EQ(strategy.getTransitionThickness(1), 1.25 * line_width) << "The transition from 1 bead to 2 happens at 125% line width.";
    EXPECT_EQ(strategy.getTransitionThickness(4), 4.25 * line_width) << "The transition from 4 beads to 5 happens at 4 + 25% line width.";

    //Transition ratio 50%.
    strategy = CenterDeviationBeadingStrategy(line_width, 0.6, 0.5);
    EXPECT_EQ(strategy.getTransitionThickness(0), 0.5 * line_width) << "The transition from 0 beads to 1 happens at 50% line width.";
    EXPECT_EQ(strategy.getTransitionThickness(1), 1.5 * line_width) << "The transition from 1 bead to 2 happens at 150% line width.";
    EXPECT_EQ(strategy.getTransitionThickness(5), 5.5 * line_width) << "The transition from 5 beads to 6 happens at 5 + 50% line width.";

    //Transition ratio 95%.
    strategy = CenterDeviationBeadingStrategy(line_width, 0.6, 0.95);
    EXPECT_EQ(strategy.getTransitionThickness(0), 0.95 * line_width) << "The transition from 0 beads to 1 happens at 95% line width.";
    EXPECT_EQ(strategy.getTransitionThickness(1), 1.95 * line_width) << "The transition from 1 bead to 2 happens at 195% line width.";
    EXPECT_EQ(strategy.getTransitionThickness(3), 3.95 * line_width) << "The transition from 3 beads to 4 happens at 3 + 95% line width.";

    //Transition ratio 100%.
    strategy = CenterDeviationBeadingStrategy(line_width, 0.6, 1);
    EXPECT_EQ(strategy.getTransitionThickness(0), line_width) << "Only transition to have a line if it fits completely.";
    EXPECT_EQ(strategy.getTransitionThickness(1), 2 * line_width) << "Only transition to have two lines if they both fit completely.";
    EXPECT_EQ(strategy.getTransitionThickness(2), 3 * line_width) << "Only transition to have three lines if they all fit completely.";

    //Transition ratio 0%.
    strategy = CenterDeviationBeadingStrategy(line_width, 0.6, 0);
    EXPECT_EQ(strategy.getTransitionThickness(0), 0) << "Always transition to 1 line. The minimum line width is 0 after all.";
    EXPECT_EQ(strategy.getTransitionThickness(1), line_width) << "If 1 line fits completely, immediately transition to 2 lines.";
    EXPECT_EQ(strategy.getTransitionThickness(6), 6 * line_width) << "If 6 lines fit completely, immediately transition to 7 lines.";
}

}