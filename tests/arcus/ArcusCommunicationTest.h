//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ARCUSCOMMUNICATION_TEST_H
#define ARCUSCOMMUNICATION_TEST_H

#include <functional> // function

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>


namespace cura
{

class ArcusCommunicationTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(ArcusCommunicationTest);
    CPPUNIT_TEST(dummyTest);
    CPPUNIT_TEST_SUITE_END();

public:
    /*!
     * \brief Sets up the test suite to prepare for testing.
     *
     * Since <em>SettingsTest</em> only has static functions, no instance
     * needs to be created here.
     */
    void setUp();

    /*!
     * \brief Tears down the test suite when testing is done.
     *
     * Since <em>SettingsTest</em> only has static functions, no instance
     * exists that needs to be destroyed.
     */
    void tearDown();

    /*!
     * \brief Test
     */
    void dummyTest();

};

}

#endif //CURAENGINE_ARCUSCOMMUNICATIONTEST_H
