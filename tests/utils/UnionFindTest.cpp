//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "UnionFindTest.h"

namespace cura
{

CPPUNIT_TEST_SUITE_REGISTRATION(UnionFindTest);

void UnionFindTest::setUp()
{
    union_find = UnionFind<char>(); //Recreate the UnionFind data structure.
}

void UnionFindTest::tearDown()
{
    //Nothing to tear down.
}

void UnionFindTest::findSimpleTest()
{
    size_t original = union_find.add('A');
    size_t result = union_find.find('A');
    CPPUNIT_ASSERT_MESSAGE("The set of the first element may not be -1.", result != (size_t)-1);
    CPPUNIT_ASSERT_MESSAGE("Find must return the original key that was returned when adding.", result == original);
}

void UnionFindTest::findMultipleTest()
{
    size_t original_a = union_find.add('A');
    size_t original_b = union_find.add('B');
    size_t original_c = union_find.add('C');
    size_t result_a = union_find.find('A');
    size_t result_b = union_find.find('B');
    size_t result_c = union_find.find('C');

    CPPUNIT_ASSERT_MESSAGE("A must be the same as when adding it.", original_a == result_a);
    CPPUNIT_ASSERT_MESSAGE("B must be the same as when adding it.", original_b == result_b);
    CPPUNIT_ASSERT_MESSAGE("C must be the same as when adding it.", original_c == result_c);
    CPPUNIT_ASSERT_MESSAGE("A must not be in the same set as B or C.", result_a != result_b && result_a != result_c);
    CPPUNIT_ASSERT_MESSAGE("B must not be in the same set as C.", result_b != result_c);
}

void UnionFindTest::uniteTwoTest()
{
    size_t a = union_find.add('A');
    size_t b = union_find.add('B');
    CPPUNIT_ASSERT_MESSAGE("A must not yet be in the same set as B.", a != b);

    union_find.unite(a, b);

    a = union_find.find('A');
    b = union_find.find('B');
    CPPUNIT_ASSERT_MESSAGE("A must now be in the same set as B.", a == b);
}

void UnionFindTest::uniteThreeTest()
{
    size_t a = union_find.add('A');
    size_t b = union_find.add('B');
    size_t c = union_find.add('C');
    CPPUNIT_ASSERT_MESSAGE("A, B and C must not yet be in the same set!", a != b && a != c && b != c);

    union_find.unite(a, b);
    union_find.unite(b, c);

    a = union_find.find('A');
    b = union_find.find('B');
    c = union_find.find('C');
    CPPUNIT_ASSERT_MESSAGE("A, B and C must now be in the same set.", a == b && b == c);
}

void UnionFindTest::uniteSetsTest()
{
    size_t a = union_find.add('A');
    size_t b = union_find.add('B');
    size_t c = union_find.add('C');
    size_t d = union_find.add('D');
    CPPUNIT_ASSERT_MESSAGE("A, B, C and D must not yet be in the same set!", a != b && a != c && a != d && b != c && b != d && c != d);

    union_find.unite(a, b);
    union_find.unite(c, d);
    //At this point we have two sets of two.

    a = union_find.find('A');
    b = union_find.find('B');
    c = union_find.find('C');
    d = union_find.find('D');
    CPPUNIT_ASSERT_MESSAGE("A and B must now be in the same set.", a == b);
    CPPUNIT_ASSERT_MESSAGE("C and D must now be in the same set.", c == d);
    CPPUNIT_ASSERT_MESSAGE("A+B and C+D must not yet be in the same set.", a != c);

    union_find.unite(a, c);
    a = union_find.find('A');
    b = union_find.find('B');
    c = union_find.find('C');
    d = union_find.find('D');
    CPPUNIT_ASSERT_MESSAGE("A, B, C and D must now be in the same set.", a == b && b == c && c == d);
}

}