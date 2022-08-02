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
class WriteInt2mmTest : public testing::TestWithParam<int>
{
};

/*
 * Parameterized test that converts the integer to millimetres, and then
 * converts it best to verify that the result was correct.
 */
TEST_P(WriteInt2mmTest, WriteInt2mm)
{
    const int in = GetParam();

    std::ostringstream ss;
    writeInt2mm(in, ss);

    ss.flush();
    const std::string str = ss.str();
    ASSERT_TRUE(ss.good()) << "The integer " << in << " was printed as '" << str << "' which was a bad string!";

    const int out = MM2INT(strtod(str.c_str(), nullptr));
    ASSERT_EQ(in, out) << "The integer " << in << " was printed as '" << str << "' which was interpreted as " << out << " rather than " << in << "!";
}

INSTANTIATE_TEST_SUITE_P(
    WriteInt2mmTestInstantiation,
    WriteInt2mmTest,
    testing::Values(-10000, -1000, -100, -10, -1, 0, 1, 10, 100, 1000, 10000, 123456789, std::numeric_limits<int32_t>::max() / 1001)); // For max integer test, divide by 1000 since MM2INT multiplies by 1000 which would cause an overflow.

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
    writeDoubleToStream(precision, in, ss);
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

INSTANTIATE_TEST_SUITE_P(WriteDoubleToStreamTestInstantiation,
                         WriteDoubleToStreamTest,
                         testing::Values(-10.000,
                                         -1.000,
                                         -0.100,
                                         -0.010,
                                         -0.001,
                                         0.010,
                                         0.100,
                                         1.000,
                                         10.000,
                                         123456.789,
                                         0.00000001,
                                         std::numeric_limits<double>::min(),
                                         std::numeric_limits<double>::max(),
                                         std::numeric_limits<double>::lowest(),
                                         -std::numeric_limits<double>::lowest()));

} // namespace cura
// NOLINTEND(*-magic-numbers)
