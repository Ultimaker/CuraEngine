//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_MAPPING_FUNCTION_TEST_H
#define UTILS_MAPPING_FUNCTION_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <../src/utils/MappingFunction.h>

namespace cura
{

class MappingFunctionTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(MappingFunctionTest);
    CPPUNIT_TEST(test_1  );
    CPPUNIT_TEST(test0   );
    CPPUNIT_TEST(test_25 );
    CPPUNIT_TEST(test_5  );
    CPPUNIT_TEST(test_75 );
    CPPUNIT_TEST(test1   );
    CPPUNIT_TEST(test1_25);
    CPPUNIT_TEST(test1_5 );
    CPPUNIT_TEST(test1_75);
    CPPUNIT_TEST(test2   );
    CPPUNIT_TEST(test2_25);
    CPPUNIT_TEST(test2_5 );
    CPPUNIT_TEST(test2_75);
    CPPUNIT_TEST(test3   );
    CPPUNIT_TEST(test5   );
    CPPUNIT_TEST_SUITE_END();

public:
    /*!
     * \brief Sets up the test suite to prepare for testing.
     * 
     * Since <em>IntPointTest</em> only has static functions, no instance
     * needs to be created here.
     */
    void setUp();

    /*!
     * \brief Tears down the test suite when testing is done.
     * 
     * Since <em>IntPointTest</em> only has static functions, no instance
     * exists that needs to be destroyed.
     */
    void tearDown();

    //These are the actual test cases. The name of the function sort of describes what it tests but I refuse to document all of these, sorry.
    void test_1  ();
    void test0   ();
    void test_25 ();
    void test_5  ();
    void test_75 ();
    void test1   ();
    void test1_25();
    void test1_5 ();
    void test1_75();
    void test2   ();
    void test2_25();
    void test2_5 ();
    void test2_75();
    void test3   ();
    void test5   ();

private:
    MappingFunction* mf;
    float delta = 0.0001;
};

}

#endif //UTILS_MAPPING_FUNCTION_TEST_H

