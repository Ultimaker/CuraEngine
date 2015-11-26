//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GCODEPLANNERTEST_H
#define GCODEPLANNERTEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "../src/gcodePlanner.h"

namespace cura
{

class GCodePlannerTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(GCodePlannerTest);
    CPPUNIT_TEST(computeNaiveTimeEstimatesRetractionTest);
    CPPUNIT_TEST_SUITE_END();
    
public:
    /*!
     * \brief Sets up the test suite to prepare for testing.
     * 
     * This creates an instance of <em>gcodePlanner</em>, ready for filling with
     * data.
     */
    void setUp();

    /*!
     * \brief Tears down the test suite when testing is done.
     * 
     * This destroys the <em>gcodePlanner</em> instance.
     */
    void tearDown();
    
    //The actual test cases.
    void computeNaiveTimeEstimatesRetractionTest();
    
private:
    /*!
     * \brief The instance of gcodePlanner that can be used for testing.
     * 
     * This instance is re-created for each test. Between tests it will be
     * <em>nullptr</em>.
     */
    GCodePlanner* gCodePlanner;
    
    /*!
     * \brief Asserts that the two time material estimates are equal.
     * 
     * If they are not equal, the error messages are formulated according to the
     * specified observed results versus the specified expected results. It will
     * include a specified string describing what test it was.
     * 
     * \param observed The observed time material estimates.
     * \param expected The expected (true) time material estimates.
     * \param test_description A description of the test that was performed to
     * get the observed results.
     */
    void verifyEstimates(const TimeMaterialEstimates& observed,const TimeMaterialEstimates& expected,std::string test_description);
};

}

#endif // GCODEPLANNERTEST_H

