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

}