//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SETTINGS_TEST_H
#define SETTINGS_TEST_H

#include <functional> // function

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "../src/settings/Settings.h"

#define DELTA 0.000000001   // Used to skip rounding errors when comparing doubles

namespace cura
{

class SettingsTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(SettingsTest);
    CPPUNIT_TEST(addSettingStringTest);
    CPPUNIT_TEST(addSettingDoubleTest);
    CPPUNIT_TEST(addSettingSizeTTest);
    CPPUNIT_TEST(addSettingBoolTest);
    CPPUNIT_TEST(addSettingExtruderTrainTest);
    CPPUNIT_TEST(addSettingLayerIndexTest);
    CPPUNIT_TEST(addSettingLayerIndexNegativeTest);
    CPPUNIT_TEST(addSettingCoordTTest);
    CPPUNIT_TEST(addSettingAngleRadiansTest);
    CPPUNIT_TEST(addSettingAngleDegreesTest);
    CPPUNIT_TEST(addSettingTemperatureTest);
    CPPUNIT_TEST(addSettingVelocityTest);
    CPPUNIT_TEST(addSettingRatioTest);
    CPPUNIT_TEST(addSettingDurationTest);
    CPPUNIT_TEST(addSettingFlowTempGraphTest);
    CPPUNIT_TEST(addSettingFMatrix3x3Test);
    CPPUNIT_TEST(addSettingVectorTest);
    CPPUNIT_TEST(overwriteSettingTest);
    CPPUNIT_TEST(inheritanceTest);
    CPPUNIT_TEST(limitToExtruderTest);
    CPPUNIT_TEST_SUITE_END();

public:
    /*
     * \brief Generate fixtures for these tests.
     */
    void setUp();

    /*!
     * \brief Test if a setting with a string value is correctly inserted
     */
    void addSettingStringTest();

    /*!
     * \brief Test if a setting with an int value is correctly inserted
     */
    void addSettingIntTest();

    /*!
     * \brief Test if a setting with a double value is correctly inserted
     */
    void addSettingDoubleTest();

    /*!
     * \brief Test if a setting with a size_t value is correctly inserted
     */
    void addSettingSizeTTest();

    /*!
     * \brief Test if a setting with an unsigned int value is correctly inserted
     */
    void addSettingUnsignedIntTest();

    /*!
     * \brief Test if a setting with an bool value is correctly inserted
     */
    void addSettingBoolTest();

    /*!
     * \brief Test if a setting with an ExtruderTrain value is correctly inserted
     */
    void addSettingExtruderTrainTest();

    /*!
     * \brief Test if a setting with a LayerIndex value is correctly inserted
     */
    void addSettingLayerIndexTest();

    /*!
     * \brief Test if a setting with a LayerIndex value is parsed correctly even
     * if the layer is negative.
     */
    void addSettingLayerIndexNegativeTest();

    /*!
     * \brief Test if a setting with a coord_t value is correctly inserted
     */
    void addSettingCoordTTest();

    /*!
     * \brief Test if a setting with a AngleRadians value is correctly inserted
     */
    void addSettingAngleRadiansTest();

    /*!
     * \brief Test if a setting with a AngleDegrees value is correctly inserted
     */
    void addSettingAngleDegreesTest();

    /*!
     * \brief Test if a setting with a Temperature value is correctly inserted
     */
    void addSettingTemperatureTest();

    /*!
     * \brief Test if a setting with a Velocity value is correctly inserted
     */
    void addSettingVelocityTest();

    /*!
     * \brief Test if a setting with a Ratio value is correctly inserted
     */
    void addSettingRatioTest();

    /*!
     * \brief Test if a setting with a Duration value is correctly inserted
     */
    void addSettingDurationTest();

    /*!
     * \brief Test if a setting with a FlowTempGraph value is correctly inserted
     */
    void addSettingFlowTempGraphTest();

    /*!
     * \brief Test if a setting with a FMatrix3x3 value is correctly inserted
     */
    void addSettingFMatrix3x3Test();

    /*!
     * \brief Test if a setting with a vector value is correctly inserted
     */
    void addSettingVectorTest();

    /*!
     * \brief Test to overwrite the value of the same setting
     */
    void overwriteSettingTest();

    /*
     * \brief Test setting inheritance from the parent setting.
     */
    void inheritanceTest();

    /*
     * \brief Test limit to extruder functionality.
     */
    void limitToExtruderTest();

private:
    Settings settings; //Settings fixture to test on.
};

}

#endif // SETTINGS_TEST_H

