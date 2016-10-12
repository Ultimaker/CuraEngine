//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef STRING_TEST_H
#define STRING_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

namespace cura
{

class StringTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(StringTest);
    CPPUNIT_TEST(writeInt2mmTest10000Negative);
    CPPUNIT_TEST(writeInt2mmTest1000Negative);
    CPPUNIT_TEST(writeInt2mmTest100Negative);
    CPPUNIT_TEST(writeInt2mmTest10Negative);
    CPPUNIT_TEST(writeInt2mmTest1Negative);
    CPPUNIT_TEST(writeInt2mmTest0);
    CPPUNIT_TEST(writeInt2mmTest1);
    CPPUNIT_TEST(writeInt2mmTest10);
    CPPUNIT_TEST(writeInt2mmTest100);
    CPPUNIT_TEST(writeInt2mmTest1000);
    CPPUNIT_TEST(writeInt2mmTest10000);
    CPPUNIT_TEST(writeInt2mmTest123456789);
    CPPUNIT_TEST_SUITE_END();

public:
    /*!
     * \brief Sets up the test suite to prepare for testing.
     * 
     * Since <em>StringTest</em> only has static functions, no instance
     * needs to be created here.
     */
    void setUp();

    /*!
     * \brief Tears down the test suite when testing is done.
     * 
     * Since <em>StringTest</em> only has static functions, no instance
     * exists that needs to be destroyed.
     */
    void tearDown();

    //These are the actual test cases. The name of the function sort of describes what it tests but I refuse to document all of these, sorry.
    void writeInt2mmTest10000Negative();
    void writeInt2mmTest1000Negative();
    void writeInt2mmTest100Negative();
    void writeInt2mmTest10Negative();
    void writeInt2mmTest1Negative();
    void writeInt2mmTest0();
    void writeInt2mmTest1();
    void writeInt2mmTest10();
    void writeInt2mmTest100();
    void writeInt2mmTest1000();
    void writeInt2mmTest10000();
    void writeInt2mmTest123456789();

private:

    /*!
     * \brief Performs the actual assertion for the getDist2FromLineSegmentTest.
     * 
     * This is essentially a parameterised version of all unit tests pertaining
     * to the writeInt2mm tests.
     * 
     * \param in the integer to check
     */
    void writeInt2mmAssert(int64_t in);
};

}

#endif //STRING_TEST_H