// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include <stdexcept>
#include <type_traits>

#include <gtest/gtest.h>

#include "types/Coord_t.h"
#include "types/Duration.h"
#include "types/Ratio.h"
#include "types/Velocity.h"

namespace curaengine
{
using namespace types;
using namespace utils;
// NOLINTBEGIN(*-magic-numbers)

TEST(CoordTTest, TestConstexprConversions)
{
    static_assert(std::is_same_v<decltype(&intToMm), double (*)(coord_t)>);
    static_assert(std::is_same_v<decltype(&intToMm2), double (*)(coord_t)>);
    static_assert(std::is_same_v<decltype(&mmToInt), coord_t (*)(double)>);
    static_assert(std::is_same_v<decltype(&mm2ToInt), coord_t (*)(double)>);
    static_assert(std::is_same_v<decltype(&mm3ToInt), coord_t (*)(double)>);
    static_assert(intToMm(1500) == 1.5);
    static_assert(intToMm2(-1500000) == -1.5);
    static_assert(mmToInt(0.0005) == 1);
    static_assert(mmToInt(-0.0005) == -1);
    static_assert(mm2ToInt(0.0000005) == 1);
    static_assert(mm2ToInt(-0.0000005) == -1);
    static_assert(mm3ToInt(0.0000000005) == 1);
    static_assert(mm3ToInt(-0.0000000005) == -1);
    static_assert(intToMm(0) == 0.0);
    static_assert(intToMm2(0) == 0.0);
    static_assert(mmToInt(0.0) == 0);
    static_assert(mm2ToInt(0.0) == 0);
    static_assert(mm3ToInt(0.0) == 0);
}

TEST(CoordTTest, TestConversionRoundingAndPrecision)
{
    const auto check_conversions = [](double n)
    {
        EXPECT_EQ(mmToInt(n), static_cast<coord_t>(n * 1000 + 0.5 * ((n > 0) - (n < 0))));
        EXPECT_EQ(mm2ToInt(n), static_cast<coord_t>(n * 1000000 + 0.5 * ((n > 0) - (n < 0))));
        EXPECT_EQ(mm3ToInt(n), static_cast<coord_t>(n * 1000000000 + 0.5 * ((n > 0) - (n < 0))));
    };
    for (double n : { 0.0, 0.0004999, -0.0004999, 0.0005, -0.0005, 0.0005001, -0.0005001, 1.2345, -1.2345 })
    {
        check_conversions(n);
        check_conversions(static_cast<double>(static_cast<float>(n)));
    }
    check_conversions(1);
    check_conversions(-1);
    check_conversions(1U);
    check_conversions(true);
    const auto check_half_thresholds = [](coord_t (*convert)(double), double half_threshold)
    {
        EXPECT_EQ(convert(half_threshold * 0.9998), 0);
        EXPECT_EQ(convert(-half_threshold * 0.9998), 0);
        EXPECT_EQ(convert(half_threshold), 1);
        EXPECT_EQ(convert(-half_threshold), -1);
        EXPECT_EQ(convert(half_threshold * 1.0002), 1);
        EXPECT_EQ(convert(-half_threshold * 1.0002), -1);
    };
    check_half_thresholds(&mmToInt, 0.0005);
    check_half_thresholds(&mm2ToInt, 0.0000005);
    check_half_thresholds(&mm3ToInt, 0.0000000005);
    check_conversions(static_cast<double>(1.2345L));
    EXPECT_EQ(mmToInt(0.0015), 2);
    EXPECT_EQ(mmToInt(-0.0015), -2);
    check_conversions(static_cast<double>(Velocity{ 1.2345 }));
    check_conversions(static_cast<double>(Ratio{ 1.2345 }));
    check_conversions(static_cast<double>(Duration{ 1.2345 }));
    const Velocity top_speed{ 50.0 };
    const Velocity start_speed{ 25.0 };
    const auto distance = (square(top_speed) - square(start_speed)) / 100.0;
    EXPECT_EQ(mmToInt(static_cast<double>(distance)), 18750);
}

TEST(CoordTTest, TestConversionsEvaluateArgumentOnce)
{
    int calls = 0;
    const auto value = [&]
    {
        ++calls;
        return 1.5;
    };
    EXPECT_EQ(intToMm(static_cast<coord_t>(value())), 0.001);
    EXPECT_EQ(intToMm2(static_cast<coord_t>(value())), 0.000001);
    EXPECT_EQ(mmToInt(value()), 1500);
    EXPECT_EQ(mm2ToInt(value()), 1500000);
    EXPECT_EQ(mm3ToInt(value()), 1500000000);
    EXPECT_EQ(calls, 5);
}

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

} // namespace curaengine
