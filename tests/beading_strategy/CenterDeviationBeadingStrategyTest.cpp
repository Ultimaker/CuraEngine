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

}