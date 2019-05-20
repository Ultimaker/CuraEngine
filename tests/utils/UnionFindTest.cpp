//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include "../src/utils/UnionFind.h"

namespace cura
{

/*
 * Fixture that contains an empty, pre-constructed union-find data structure of
 * characters.
 */
class UnionFindTest : public testing::Test
{
public:
    UnionFind<char> union_find;
};

TEST_F(UnionFindTest, FindSimple)
{
    const size_t original = union_find.add('A');
    const size_t result = union_find.find('A');

    EXPECT_NE(result, (size_t)-1) << "The set of the first element may not be -1.";
    ASSERT_EQ(result, original) << "Find must return the original key that was returned when adding.";
}

TEST_F(UnionFindTest, FindMultiple)
{
    const size_t original_a = union_find.add('A');
    const size_t original_b = union_find.add('B');
    const size_t original_c = union_find.add('C');
    const size_t result_a = union_find.find('A');
    const size_t result_b = union_find.find('B');
    const size_t result_c = union_find.find('C');

    EXPECT_EQ(original_a, result_a) << "A must be the same as when adding it.";
    EXPECT_EQ(original_b, result_b) << "B must be the same as when adding it.";
    EXPECT_EQ(original_c, result_c) << "C must be the same as when adding it.";
    EXPECT_NE(result_a, result_b) << "A must not be in the same set as B.";
    EXPECT_NE(result_a, result_c) << "A must not be in the same set as C.";
    EXPECT_NE(result_b, result_c) << "B must not be in the same set as C.";
}

TEST_F(UnionFindTest, UniteTwo)
{
    size_t a = union_find.add('A');
    size_t b = union_find.add('B');
    ASSERT_NE(a, b) << "A must not yet be in the same set as B.";

    union_find.unite(a, b);

    a = union_find.find('A');
    b = union_find.find('B');
    ASSERT_EQ(a, b) << "A must now be in the same set as B.";
}

TEST_F(UnionFindTest, UniteThree)
{
    size_t a = union_find.add('A');
    size_t b = union_find.add('B');
    size_t c = union_find.add('C');
    ASSERT_NE(a, b) << "A and B must not yet be in the same set!";
    ASSERT_NE(a, c) << "A and C must not yet be in the same set!";
    ASSERT_NE(b, c) << "B and C must not yet be in the same set!";

    union_find.unite(a, b);
    union_find.unite(b, c);

    a = union_find.find('A');
    b = union_find.find('B');
    c = union_find.find('C');
    ASSERT_EQ(a, b) << "A and B must now be in the same set.";
    ASSERT_EQ(b, c) << "B and C must now be in the same set.";
}

TEST_F(UnionFindTest, UniteSets)
{
    size_t a = union_find.add('A');
    size_t b = union_find.add('B');
    size_t c = union_find.add('C');
    size_t d = union_find.add('D');
    ASSERT_NE(a, b) << "A and B must not yet be in the same set!";
    ASSERT_NE(a, c) << "A and C must not yet be in the same set!";
    ASSERT_NE(a, d) << "A and D must not yet be in the same set!";
    ASSERT_NE(b, c) << "B and C must not yet be in the same set!";
    ASSERT_NE(b, d) << "B and D must not yet be in the same set!";
    ASSERT_NE(c, d) << "C and D must not yet be in the same set!";

    union_find.unite(a, b);
    union_find.unite(c, d);
    //At this point we have two sets of two.

    a = union_find.find('A');
    b = union_find.find('B');
    c = union_find.find('C');
    d = union_find.find('D');
    ASSERT_EQ(a, b) << "A and B must now be in the same set.";
    ASSERT_EQ(c, d) << "C and D must now be in the same set.";
    ASSERT_NE(a, c) << "A+B and C+D must not yet be in the same set.";

    union_find.unite(a, c);
    a = union_find.find('A');
    b = union_find.find('B');
    c = union_find.find('C');
    d = union_find.find('D');
    ASSERT_EQ(a, b) << "A and B must still be in the same set.";
    ASSERT_EQ(c, d) << "C and D must still be in the same set.";
    ASSERT_EQ(b, c) << "A+B and C+D must now be in the same set.";
}

}