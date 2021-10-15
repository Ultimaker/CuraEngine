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
    CenterDeviationBeadingStrategy strategy(400, 0.6, 0.5, 0.5); //Pretty standard settings.
    EXPECT_EQ(strategy.minimum_line_width_split, 200) << "Since the transition threshold was 50%, the minimum line width should be 0.5 times the preferred width.";
    strategy = CenterDeviationBeadingStrategy(400, 0.6, 0, 0);
    EXPECT_EQ(strategy.minimum_line_width_split, 0) << "Since the transition threshold was 0%, the minimum line width should be 0.";
    strategy = CenterDeviationBeadingStrategy(0, 0.6, 0.5, 0.5);
    EXPECT_EQ(strategy.minimum_line_width_split, 0) << "Since the line width was 0%, the minimum line width should also be 0.";
    strategy = CenterDeviationBeadingStrategy(400, 0.6, 1, 1);
    EXPECT_EQ(strategy.minimum_line_width_split, 400) << "Since the transition threshold was 100%, the minimum line width equals the preferred width.";
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
    CenterDeviationBeadingStrategy strategy(line_width, 0.6, 0.5, 0.5);
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
    CenterDeviationBeadingStrategy strategy(line_width, 0.6, 0.25, 0.25);
    EXPECT_EQ(strategy.getTransitionThickness(0), 0.25 * line_width) << "The transition from 0 beads to 1 happens at 25% line width.";
    EXPECT_EQ(strategy.getTransitionThickness(1), 1.25 * line_width) << "The transition from 1 bead to 2 happens at 125% line width.";
    EXPECT_EQ(strategy.getTransitionThickness(4), 4.25 * line_width) << "The transition from 4 beads to 5 happens at 4 + 25% line width.";

    //Transition ratio 50%.
    strategy = CenterDeviationBeadingStrategy(line_width, 0.6, 0.5, 0.5);
    EXPECT_EQ(strategy.getTransitionThickness(0), 0.5 * line_width) << "The transition from 0 beads to 1 happens at 50% line width.";
    EXPECT_EQ(strategy.getTransitionThickness(1), 1.5 * line_width) << "The transition from 1 bead to 2 happens at 150% line width.";
    EXPECT_EQ(strategy.getTransitionThickness(5), 5.5 * line_width) << "The transition from 5 beads to 6 happens at 5 + 50% line width.";

    //Transition ratio 95%.
    strategy = CenterDeviationBeadingStrategy(line_width, 0.6, 0.95, 0.95);
    EXPECT_EQ(strategy.getTransitionThickness(0), 0.95 * line_width) << "The transition from 0 beads to 1 happens at 95% line width.";
    EXPECT_EQ(strategy.getTransitionThickness(1), 1.95 * line_width) << "The transition from 1 bead to 2 happens at 195% line width.";
    EXPECT_EQ(strategy.getTransitionThickness(3), 3.95 * line_width) << "The transition from 3 beads to 4 happens at 3 + 95% line width.";

    //Transition ratio 100%.
    strategy = CenterDeviationBeadingStrategy(line_width, 0.6, 1, 1);
    EXPECT_EQ(strategy.getTransitionThickness(0), line_width) << "Only transition to have a line if it fits completely.";
    EXPECT_EQ(strategy.getTransitionThickness(1), 2 * line_width) << "Only transition to have two lines if they both fit completely.";
    EXPECT_EQ(strategy.getTransitionThickness(2), 3 * line_width) << "Only transition to have three lines if they all fit completely.";

    //Transition ratio 0%.
    strategy = CenterDeviationBeadingStrategy(line_width, 0.6, 0, 0);
    EXPECT_EQ(strategy.getTransitionThickness(0), 0) << "Always transition to 1 line. The minimum line width is 0 after all.";
    EXPECT_EQ(strategy.getTransitionThickness(1), line_width) << "If 1 line fits completely, immediately transition to 2 lines.";
    EXPECT_EQ(strategy.getTransitionThickness(6), 6 * line_width) << "If 6 lines fit completely, immediately transition to 7 lines.";
}

/*!
 * Test getting the optimal bead count for a given shape width.
 */
TEST(CenterDeviationBeadingStrategy, GetOptimalBeadCount)
{
    constexpr coord_t line_width = 400;

    //Transition ratio 25%.
    CenterDeviationBeadingStrategy strategy(line_width, 0.6, 0.25, 0.25);
    //Anything below 25% line width should then produce 0 lines.
    for(coord_t width = 0; width < 0.25 * line_width; width += 10)
    {
        EXPECT_EQ(strategy.getOptimalBeadCount(width), 0) << "Width is below the transition thickness from 0 to 1 line, so it should produce 0 lines.";
    }
    EXPECT_LE(strategy.getOptimalBeadCount(0.25 * line_width), 1) << "At exactly the transition thickness, either 0 or 1 line is acceptable.";
    for(coord_t width = 0.25 * line_width + 1; width < 1.25 * line_width; width += 10)
    {
        EXPECT_EQ(strategy.getOptimalBeadCount(width), 1) << "Width is above the transition thickness from 0 to 1 line but below 1 to 2 lines, so it should produce 1 line.";
    }
    EXPECT_TRUE(strategy.getOptimalBeadCount(1.25 * line_width) == 1 || strategy.getOptimalBeadCount(1.25 * line_width) == 2) << "At exactly 125% line width, either 1 or 2 lines is acceptable.";
    for(coord_t width = 1.25 * line_width + 1; width < 2.25 * line_width; width += 10)
    {
        EXPECT_EQ(strategy.getOptimalBeadCount(width), 2) << "Width is above the transition thickness from 1 to 2 lines but below 2 to 3 lines, so it should produce 2 lines.";
    }

    //Transition ratio 80%.
    strategy = CenterDeviationBeadingStrategy(line_width, 0.6, 0.8, 0.8);
    //Anything below 80% line width should then produce 0 lines.
    for(coord_t width = 0; width < 0.8 * line_width; width += 10)
    {
        EXPECT_EQ(strategy.getOptimalBeadCount(width), 0) << "Width is below the transition thickness from 0 to 1 line, so it should produce 0 lines.";
    }
    EXPECT_LE(strategy.getOptimalBeadCount(0.8 * line_width), 1) << "At exactly the transition thickness, either 0 or 1 line is acceptable.";
    for(coord_t width = 0.8 * line_width + 1; width < 1.8 * line_width; width += 10)
    {
        EXPECT_EQ(strategy.getOptimalBeadCount(width), 1) << "Width is above the transition thickness from 0 to 1 line but below 1 to 2 lines, so it should produce 1 line.";
    }
    EXPECT_TRUE(strategy.getOptimalBeadCount(1.8 * line_width) == 1 || strategy.getOptimalBeadCount(1.8 * line_width) == 2) << "At exactly 180% line width, either 1 or 2 lines is acceptable.";
    for(coord_t width = 1.8 * line_width + 1; width < 2.8 * line_width; width += 10)
    {
        EXPECT_EQ(strategy.getOptimalBeadCount(width), 2) << "Width is above the transition thickness from 1 to 2 lines but below 2 to 3 lines, so it should produce 2 lines.";
    }
}

/*!
 * Tests whether the line compactness setting does what it is supposed to do,
 * producing fewer, wider lines when the setting is high than when the setting
 * is low.
 *
 * This is a test for requirements. The exact outcome of a function is not
 * tested, but properties of the outcome is tested.
 */
TEST(CenterDeviationBeadingStrategy, LineCompactnessMonotonic)
{
    constexpr coord_t line_width = 400;
    constexpr coord_t widths[] = {0, 1, 99, 101, 150, 200, 299, 300, 301, 399, 400, 401, 410, 450, 500, 660, 770, 880, 910, 1000, 1200}; //Bunch of widths to test with.
    constexpr float compactnesses[] = {0, 0.1, 0.2, 0.24, 0.25, 0.26, 0.3, 0.5, 0.7, 0.75, 0.99, 1}; //Bunch of line compactness factors to test with.
    constexpr size_t num_compactnesses = sizeof(compactnesses) / sizeof(float);

    for(coord_t width : widths)
    {
        for(size_t low_index = 0; low_index < num_compactnesses; ++low_index)
        {
            const float low_compactness = compactnesses[low_index];
            for(size_t high_index = low_index; high_index < num_compactnesses; ++high_index)
            {
                const float high_compactness = compactnesses[high_index];

                EXPECT_GE(
                        CenterDeviationBeadingStrategy(line_width, 0.6, low_compactness, low_compactness).getOptimalBeadCount(width),
                        CenterDeviationBeadingStrategy(line_width, 0.6, high_compactness, high_compactness).getOptimalBeadCount(width)
                ) << "When the compactness is low, the number of beads should always be greater or equal to when the compactness is high.";
            }
        }
    }
}

}