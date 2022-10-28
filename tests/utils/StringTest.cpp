// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/string.h" // The file under test.
#include "utils/IntPoint.h"
#include <gtest/gtest.h>

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{

/*
 * Fixture to allow parameterized tests for writeInt2mm.
 */
class WriteInt2mmTest : public testing::TestWithParam<coord_t>
{
};

/*
 * Parameterized test that converts the integer to millimetres, and then
 * converts it best to verify that the result was correct.
 */
TEST_P(WriteInt2mmTest, WriteInt2mm)
{
    const coord_t in = GetParam();

    std::ostringstream ss;
    ss << MMtoStream{ in };

    ss.flush();
    const std::string str = ss.str();
    ASSERT_TRUE(ss.good()) << "The integer " << in << " was printed as '" << str << "' which was a bad string!";

    const coord_t out = mm_to_coord(std::stod(str));
    ASSERT_EQ(in, out) << "The integer " << in << " was printed as '" << str << "' which was interpreted as " << out << " rather than " << in << "!";
}

// Max coord_t value that can be parsed exactly by std::stod 0x7ffffffffffff800
constexpr coord_t max_integer_as_float = static_cast<double>(std::numeric_limits<coord_t>::max()) * (1 - std::numeric_limits<double>::epsilon());

INSTANTIATE_TEST_SUITE_P(WriteInt2mmTestInstantiation, WriteInt2mmTest, testing::Values(-10000, -1000, -100, -10, -1, 0, 1, 10, 100, 1000, 10000, 123456789, max_integer_as_float, -max_integer_as_float));

/*
 * Fixture to allow parameterized tests for writeDoubleToStream.
 */
class WriteDoubleToStreamTest : public testing::TestWithParam<double>
{
};

TEST_P(WriteDoubleToStreamTest, WriteDoubleToStream)
{
    const double in = GetParam();
    constexpr unsigned int precision = 4;

    std::ostringstream ss;
    ss << PrecisionedDouble<precision>{ in };

    ss.flush();
    const std::string str = ss.str();

    ASSERT_TRUE(ss.good()) << "The double " << in << " was printed as '" << str << " which was a bad string!";

    const double out = strtod(str.c_str(), nullptr);
    std::ostringstream in_ss;
    in_ss << std::fixed << std::setprecision(precision) << in;
    const std::string in_str = in_ss.str();
    const double in_reinterpreted = strtod(in_str.c_str(), nullptr);

    ASSERT_EQ(in_reinterpreted, out) << "The double " << in << " was printed as '" << str << "' which was interpreted as " << out << " rather than " << in_reinterpreted << "!";
}

INSTANTIATE_TEST_SUITE_P(WriteDoubleToStreamTestInstantiation, WriteDoubleToStreamTest, testing::Values(-10.000, -1.000, -0.100, -0.010, -0.001, 0.010, 0.100, 1.000, 10.000, 123456.789, 0.00000001));

} // namespace cura
// NOLINTEND(*-magic-numbers)
