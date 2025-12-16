// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include <stdexcept>

#include <gtest/gtest.h>

#include "utils/Coord_t.h"

namespace cura
{
// NOLINTBEGIN(*-magic-numbers)

TEST(CoordTTest, TestFuzzyZero)
{
    EXPECT_EQ(fuzzy_is_zero(0), true);
    EXPECT_EQ(fuzzy_is_zero(2), true);
    EXPECT_EQ(fuzzy_is_zero(-2), true);
    EXPECT_EQ(fuzzy_is_zero(10), false);
    EXPECT_EQ(fuzzy_is_zero(-10), false);
}

TEST(CoordTTest, TestFuzzyEqual)
{
    EXPECT_EQ(fuzzy_equal(0, 0), true);
    EXPECT_EQ(fuzzy_equal(0, 3), true);
    EXPECT_EQ(fuzzy_equal(-2, 2), true);
    EXPECT_EQ(fuzzy_equal(-2, 5), false);

    EXPECT_EQ(fuzzy_equal(100, 100), true);
    EXPECT_EQ(fuzzy_equal(100, 102), true);
    EXPECT_EQ(fuzzy_equal(100, 97), true);
    EXPECT_EQ(fuzzy_equal(100, 110), false);
    EXPECT_EQ(fuzzy_equal(100, 90), false);

    EXPECT_EQ(fuzzy_equal(-100, -100), true);
    EXPECT_EQ(fuzzy_equal(-100, -102), true);
    EXPECT_EQ(fuzzy_equal(-100, -97), true);
    EXPECT_EQ(fuzzy_equal(-100, -110), false);
    EXPECT_EQ(fuzzy_equal(-100, -90), false);
}

TEST(CoordTTest, TestFuzzyNotEqual)
{
    EXPECT_EQ(fuzzy_not_equal(0, 0), false);
    EXPECT_EQ(fuzzy_not_equal(0, 3), false);
    EXPECT_EQ(fuzzy_not_equal(-2, 2), false);
    EXPECT_EQ(fuzzy_not_equal(-2, 5), true);

    EXPECT_EQ(fuzzy_not_equal(100, 100), false);
    EXPECT_EQ(fuzzy_not_equal(100, 102), false);
    EXPECT_EQ(fuzzy_not_equal(100, 97), false);
    EXPECT_EQ(fuzzy_not_equal(100, 110), true);
    EXPECT_EQ(fuzzy_not_equal(100, 90), true);

    EXPECT_EQ(fuzzy_not_equal(-100, -100), false);
    EXPECT_EQ(fuzzy_not_equal(-100, -102), false);
    EXPECT_EQ(fuzzy_not_equal(-100, -97), false);
    EXPECT_EQ(fuzzy_not_equal(-100, -110), true);
    EXPECT_EQ(fuzzy_not_equal(-100, -90), true);
}

TEST(CoordTTest, TestFuzzyGreater)
{
    EXPECT_EQ(fuzzy_is_greater(0, 0), false);
    EXPECT_EQ(fuzzy_is_greater(0, 3), false);
    EXPECT_EQ(fuzzy_is_greater(-2, 2), false);
    EXPECT_EQ(fuzzy_is_greater(-2, 5), false);
    EXPECT_EQ(fuzzy_is_greater(5, -2), true);
    EXPECT_EQ(fuzzy_is_greater(10, 1), true);

    EXPECT_EQ(fuzzy_is_greater(100, 100), false);
    EXPECT_EQ(fuzzy_is_greater(100, 102), false);
    EXPECT_EQ(fuzzy_is_greater(100, 97), false);
    EXPECT_EQ(fuzzy_is_greater(100, 110), false);
    EXPECT_EQ(fuzzy_is_greater(100, 90), true);

    EXPECT_EQ(fuzzy_is_greater(-100, -100), false);
    EXPECT_EQ(fuzzy_is_greater(-100, -102), false);
    EXPECT_EQ(fuzzy_is_greater(-100, -97), false);
    EXPECT_EQ(fuzzy_is_greater(-100, -110), true);
    EXPECT_EQ(fuzzy_is_greater(-100, -90), false);
}

TEST(CoordTTest, TestFuzzyGreaterOrEqual)
{
    EXPECT_EQ(fuzzy_is_greater_or_equal(0, 0), true);
    EXPECT_EQ(fuzzy_is_greater_or_equal(0, 3), true);
    EXPECT_EQ(fuzzy_is_greater_or_equal(-2, 2), true);
    EXPECT_EQ(fuzzy_is_greater_or_equal(-2, 5), false);
    EXPECT_EQ(fuzzy_is_greater_or_equal(5, -2), true);
    EXPECT_EQ(fuzzy_is_greater_or_equal(10, 1), true);
    EXPECT_EQ(fuzzy_is_greater_or_equal(1, 10), false);

    EXPECT_EQ(fuzzy_is_greater_or_equal(100, 100), true);
    EXPECT_EQ(fuzzy_is_greater_or_equal(100, 102), true);
    EXPECT_EQ(fuzzy_is_greater_or_equal(100, 97), true);
    EXPECT_EQ(fuzzy_is_greater_or_equal(100, 110), false);
    EXPECT_EQ(fuzzy_is_greater_or_equal(100, 90), true);

    EXPECT_EQ(fuzzy_is_greater_or_equal(-100, -100), true);
    EXPECT_EQ(fuzzy_is_greater_or_equal(-100, -102), true);
    EXPECT_EQ(fuzzy_is_greater_or_equal(-100, -97), true);
    EXPECT_EQ(fuzzy_is_greater_or_equal(-100, -110), true);
    EXPECT_EQ(fuzzy_is_greater_or_equal(-100, -90), false);
}

TEST(CoordTTest, TestFuzzyLesser)
{
    EXPECT_EQ(fuzzy_is_lesser(0, 0), false);
    EXPECT_EQ(fuzzy_is_lesser(0, 3), false);
    EXPECT_EQ(fuzzy_is_lesser(-2, 2), false);
    EXPECT_EQ(fuzzy_is_lesser(-2, 5), true);
    EXPECT_EQ(fuzzy_is_lesser(5, -2), false);
    EXPECT_EQ(fuzzy_is_lesser(10, 1), false);

    EXPECT_EQ(fuzzy_is_lesser(100, 100), false);
    EXPECT_EQ(fuzzy_is_lesser(100, 102), false);
    EXPECT_EQ(fuzzy_is_lesser(100, 97), false);
    EXPECT_EQ(fuzzy_is_lesser(100, 110), true);
    EXPECT_EQ(fuzzy_is_lesser(100, 90), false);

    EXPECT_EQ(fuzzy_is_lesser(-100, -100), false);
    EXPECT_EQ(fuzzy_is_lesser(-100, -102), false);
    EXPECT_EQ(fuzzy_is_lesser(-100, -97), false);
    EXPECT_EQ(fuzzy_is_lesser(-100, -110), false);
    EXPECT_EQ(fuzzy_is_lesser(-100, -90), true);
}

TEST(CoordTTest, TestFuzzyLesserOrEqual)
{
    EXPECT_EQ(fuzzy_is_lesser_or_equal(0, 0), true);
    EXPECT_EQ(fuzzy_is_lesser_or_equal(0, 3), true);
    EXPECT_EQ(fuzzy_is_lesser_or_equal(-2, 2), true);
    EXPECT_EQ(fuzzy_is_lesser_or_equal(-2, 5), true);
    EXPECT_EQ(fuzzy_is_lesser_or_equal(5, -2), false);
    EXPECT_EQ(fuzzy_is_lesser_or_equal(10, 1), false);
    EXPECT_EQ(fuzzy_is_lesser_or_equal(1, 10), true);

    EXPECT_EQ(fuzzy_is_lesser_or_equal(100, 100), true);
    EXPECT_EQ(fuzzy_is_lesser_or_equal(100, 102), true);
    EXPECT_EQ(fuzzy_is_lesser_or_equal(100, 97), true);
    EXPECT_EQ(fuzzy_is_lesser_or_equal(100, 110), true);
    EXPECT_EQ(fuzzy_is_lesser_or_equal(100, 90), false);

    EXPECT_EQ(fuzzy_is_lesser_or_equal(-100, -100), true);
    EXPECT_EQ(fuzzy_is_lesser_or_equal(-100, -102), true);
    EXPECT_EQ(fuzzy_is_lesser_or_equal(-100, -97), true);
    EXPECT_EQ(fuzzy_is_lesser_or_equal(-100, -110), false);
    EXPECT_EQ(fuzzy_is_lesser_or_equal(-100, -90), true);
}

// NOLINTEND(*-magic-numbers)

} // namespace cura
