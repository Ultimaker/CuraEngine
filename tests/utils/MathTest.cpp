// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/math.h"

#include <stdexcept>

#include <gtest/gtest.h>

namespace cura
{
// NOLINTBEGIN(*-magic-numbers)

// Test cases for the square function
TEST(MathTest, TestSquare)
{
    EXPECT_EQ(square(2), 4) << "Square of 2 should be 4.";
    EXPECT_EQ(square(-3), 9) << "Square of -3 should be 9.";
    EXPECT_EQ(square(0), 0) << "Square of 0 should be 0.";
    EXPECT_EQ(square(1.5), 2.25) << "Square of 1.5 should be 2.25.";
}

// Test cases for the round_divide_signed function
TEST(MathTest, TestRoundDivideSigned)
{
    EXPECT_EQ(round_divide_signed(10, 3), 3) << "10 / 3 rounded should be 3.";
    EXPECT_EQ(round_divide_signed(10, -3), -3) << "10 / -3 rounded should be -3.";
    EXPECT_EQ(round_divide_signed(-10, 3), -3) << "-10 / 3 rounded should be -3.";
    EXPECT_EQ(round_divide_signed(-10, -3), 3) << "-10 / -3 rounded should be 3.";
}

// Test cases for the ceil_divide_signed function
TEST(MathTest, TestCeilDivideSigned)
{
    EXPECT_EQ(ceil_divide_signed(10, 3), 4) << "10 / 3 rounded up should be 4.";
    EXPECT_EQ(ceil_divide_signed(10, -3), -3) << "10 / -3 rounded up should be -3.";
    EXPECT_EQ(ceil_divide_signed(-10, 3), -3) << "-10 / 3 rounded up should be -3.";
    EXPECT_EQ(ceil_divide_signed(-10, -3), 4) << "-10 / -3 rounded up should be 4.";
}

// Test cases for the floor_divide_signed function
TEST(MathTest, TestFloorDivideSigned)
{
    EXPECT_EQ(floor_divide_signed(10, 3), 3) << "10 / 3 rounded down should be 3.";
    EXPECT_EQ(floor_divide_signed(10, -3), -4) << "10 / -3 rounded down should be -4.";
    EXPECT_EQ(floor_divide_signed(-10, 3), -4) << "-10 / 3 rounded down should be -4.";
    EXPECT_EQ(floor_divide_signed(-10, -3), 3) << "-10 / -3 rounded down should be 3.";
}

// Test cases for the round_divide function
TEST(MathTest, TestRoundDivide)
{
    EXPECT_EQ(round_divide(10, 3), 3) << "10 / 3 rounded should be 3.";
    EXPECT_EQ(round_divide(11, 3), 4) << "11 / 3 rounded should be 4.";
    EXPECT_EQ(round_divide(9, 3), 3) << "9 / 3 rounded should be 3.";

}

// Test cases for the round_up_divide function
TEST(MathTest, TestRoundUpDivide)
{
    EXPECT_EQ(round_up_divide(10, 3), 4) << "10 / 3 rounded up should be 4.";
    EXPECT_EQ(round_up_divide(9, 3), 3) << "9 / 3 rounded up should be 3.";
    EXPECT_EQ(round_up_divide(1, 1), 1) << "1 / 1 rounded up should be 1.";
}

// NOLINTEND(*-magic-numbers)

} // namespace cura
