//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.
//Use this file as a template for testing new classes

#ifndef SETTINGS_TEST_H
#define SETTINGS_TEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include <../src/settings/settings.h>
#include <../src/settings/SettingRegistry.h>

namespace cura
{

class SettingsTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(SettingsTest);
    CPPUNIT_TEST(smokeTest);
    CPPUNIT_TEST(inheritBaseTest);
    CPPUNIT_TEST(getSettingTest);
    CPPUNIT_TEST(getSettingTest2);
    CPPUNIT_TEST(getSettingTest3);
    CPPUNIT_TEST(getSettingTest4);
    CPPUNIT_TEST(getFlowTempGraphTest);
    CPPUNIT_TEST(getFlowTempGraphTest2);
    CPPUNIT_TEST(getMatrixTest);
    CPPUNIT_TEST(getMatrixTest2);
    CPPUNIT_TEST(getIntegerListTest);
    CPPUNIT_TEST(getIntegerListTest2);
    CPPUNIT_TEST_SUITE_END();

public:
    void setUp();

    void tearDown();

    void smokeTest();
    void inheritBaseTest();
    void getSettingTest();
    void getSettingTest2();
    void getSettingTest3();
    void getSettingTest4();
    void getFlowTempGraphTest();
    void getFlowTempGraphTest2();
    void getMatrixTest();
    void getMatrixTest2();
    void getIntegerListTest();
    void getIntegerListTest2();

private:
    /*!
     * \brief The maximum allowed error in floats.
     */
    static constexpr double epsilon = 0.000001f;

    SettingsBase my_settings;
};

}

#endif //SETTINGS_TEST_H

