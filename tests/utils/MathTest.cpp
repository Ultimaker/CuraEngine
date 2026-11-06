// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/math.h"

#include <atomic>
#include <stdexcept>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "utils/macros.h"

namespace curaengine
{
using namespace types;
using namespace utils;
// NOLINTBEGIN(*-magic-numbers)

TEST(MathTest, TestMathematicalConstants)
{
    static_assert(tau == 2.0 * std::numbers::pi);
    EXPECT_EQ(one_over_sqrt_2, std::sqrt(1.0 / 2.0));
    EXPECT_EQ(one_over_sqrt_3, std::sqrt(1.0 / 3.0));
    EXPECT_EQ(one_over_sqrt_6, std::sqrt(1.0 / 6.0));
    EXPECT_EQ(sqrt_two_third, std::sqrt(2.0 / 3.0));
    EXPECT_EQ(sqrt_three_fourths, std::sqrt(3.0 / 4.0));
}

TEST(MathTest, TestUnusedParameterDoesNotCopy)
{
    struct NonCopyable
    {
        NonCopyable() = default;
        NonCopyable(const NonCopyable&) = delete;
    };
    NonCopyable value;
    unused_param(value);
    unused_param(NonCopyable{});
}

TEST(MathTest, TestRunOncePerCallSite)
{
    int first = 0;
    int second = 0;
    for (int i = 0; i < 10; ++i)
    {
        run_once(
            [&]
            {
                ++first;
            });
        run_once(
            [&]
            {
                ++second;
            });
    }
    EXPECT_EQ(first, 1);
    EXPECT_EQ(second, 1);
}

TEST(MathTest, TestRunOnceConcurrentCalls)
{
    std::atomic<int> calls = 0;
    std::atomic<bool> start = false;
    std::vector<std::thread> threads;
    for (int i = 0; i < 8; ++i)
    {
        threads.emplace_back(
            [&]
            {
                while (! start.load())
                {
                    std::this_thread::yield();
                }
                for (int j = 0; j < 100; ++j)
                {
                    run_once(
                        [&]
                        {
                            ++calls;
                        });
                }
            });
    }
    start = true;
    for (auto& thread : threads)
    {
        thread.join();
    }
    EXPECT_EQ(calls.load(), 1);
}

TEST(MathTest, TestRunOnceInTemplateContext)
{
    int calls = 0;
    const auto invoke = [&]<typename T>()
    {
        run_once(
            [&]
            {
                ++calls;
            });
    };
    invoke.operator()<int>();
    invoke.operator()<int>();
    invoke.operator()<double>();
    invoke.operator()<double>();
    EXPECT_EQ(calls, 2);
}

TEST(MathTest, TestRunOnceDoesNotRetryExceptions)
{
    int calls = 0;
    const auto invoke = [&]
    {
        run_once(
            [&]
            {
                ++calls;
                throw std::runtime_error("once");
            });
    };
    EXPECT_THROW(invoke(), std::runtime_error);
    EXPECT_NO_THROW(invoke());
    EXPECT_EQ(calls, 1);
}

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

} // namespace curaengine
