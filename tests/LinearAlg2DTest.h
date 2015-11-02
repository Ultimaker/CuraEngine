//Copyright (c) 2015 Ultimaker B.V.
//UltiScanTastic is released under the terms of the AGPLv3 or higher.

#ifndef LINEARALG2DTEST_H
#define	LINEARALG2DTEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

namespace cura
{

class LinearAlg2DTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(LinearAlg2DTest);
    CPPUNIT_TEST(getDist2FromLineSegmentTest);
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
    
    /*!
     * \brief Tests the LinearAlg2D#getDist2FromLineSegment function.
     */
    void getDist2FromLineSegmentTest();

};

}

#endif //LINEARALG2DTEST_H

