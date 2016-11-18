//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef POLYGON_TEST_H
#define POLYGON_TEST_H

#include <functional> // function

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "../src/utils/intpoint.h"
#include "../src/utils/polygon.h"

namespace cura
{

class PolygonTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(PolygonTest);
    CPPUNIT_TEST(polygonOffsetTest);
    CPPUNIT_TEST(polygonOffsetBugTest);
    CPPUNIT_TEST(isOutsideTest);
    CPPUNIT_TEST(isInsideTest);
    CPPUNIT_TEST_SUITE_END();

public:
    /*!
     * \brief Sets up the test suite to prepare for testing.
     * 
     * Since <em>LinearAlg2DTest</em> only has static functions, no instance
     * needs to be created here.
     */
    void setUp();

    /*!
     * \brief Tears down the test suite when testing is done.
     * 
     * Since <em>LinearAlg2DTest</em> only has static functions, no instance
     * exists that needs to be destroyed.
     */
    void tearDown();
    
    //These are the actual test cases. The name of the function sort of describes what it tests but I refuse to document all of these, sorry.
    void polygonOffsetTest();
    void polygonOffsetBugTest();
    void isOutsideTest();
    void isInsideTest();


private:
    /*!
     * \brief The maximum allowed error in distance measurements.
     */
    static const int64_t maximum_error = 10;
    
    Polygon test_square;
    Polygon pointy_square;
    Polygon triangle;
    Polygon clipper_bug;
};

}

#endif // POLYGON_TEST_H

