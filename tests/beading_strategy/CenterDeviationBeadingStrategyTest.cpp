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

}