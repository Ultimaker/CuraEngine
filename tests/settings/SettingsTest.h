//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SETTINGS_TEST_H
#define SETTINGS_TEST_H

#include <functional> // function

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "../src/settings/Setting.h"
#include "../src/settings/Settings.h"
#include "../src/settings/types/LayerIndex.h"
#include "../src/settings/types/AngleRadians.h"
#include "../src/settings/types/AngleDegrees.h"
#include "../src/settings/types/Temperature.h"
#include "../src/settings/types/Velocity.h"
#include "../src/settings/types/Ratio.h"
#include "../src/settings/types/Duration.h"
#include "../src/FlowTempGraph.h"
#include "../src/utils/floatpoint.h"

#define DELTA 0.000000001   // Used to skip rounding errors when comparing doubles

namespace cura
{

class SettingsTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(SettingsTest);
    CPPUNIT_TEST(addSettingStringTest);
    CPPUNIT_TEST(addSettingIntTest);
    CPPUNIT_TEST(addSettingDoubleTest);
    CPPUNIT_TEST(addSettingSizeTTest);
    CPPUNIT_TEST(addSettingUnsignedIntTest);
    CPPUNIT_TEST(addSettingBoolTest);
    CPPUNIT_TEST(addSettingExtruderTrainTest);
    CPPUNIT_TEST(addSettingLayerIndexTest);
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

private:

    // Some fixtures
    Settings settings;
    std::string setting_key_string;
    std::string setting_value_string;
    std::string setting_key_int;
    int setting_value_int;
    std::string setting_value_int_string;
    std::string setting_key_int2;
    int setting_value_int2;
    std::string setting_value_int_string2;
    std::string setting_key_double;
    double setting_value_double;
    std::string setting_value_double_string;
    std::string setting_key_size_t;
    std::size_t setting_value_size_t;
    std::string setting_value_size_t_string;
    std::string setting_key_unsigned_int;
    unsigned int setting_value_unsigned_int;
    std::string setting_value_unsigned_int_string;
    std::string setting_key_bool;
    bool setting_value_bool;
    std::string setting_value_bool_string;
    std::string setting_key_bool2;
    bool setting_value_bool2;
    std::string setting_value_bool_string2;
    std::string setting_key_bool3;
    bool setting_value_bool3;
    std::string setting_value_bool_string3;
    std::string setting_key_bool4;
    bool setting_value_bool4;
    std::string setting_value_bool_string4;
    std::string setting_key_bool5;
    bool setting_value_bool5;
    std::string setting_value_bool_string5;
    std::string setting_key_layerindex;
    LayerIndex setting_value_layerindex = 0;    // Set initial value here since there is not default constructor
    std::string setting_value_layerindex_string;
    std::string setting_key_coord_t;
    coord_t setting_value_coord_t;
    std::string setting_value_coord_t_string;
    std::string setting_key_angleradians;
    AngleRadians setting_value_angleradians = 0;    // Set initial value here since there is not default constructor
    std::string setting_value_angleradians_string;
    std::string setting_key_angledegrees;
    AngleDegrees setting_value_angledegrees = 0;    // Set initial value here since there is not default constructor
    std::string setting_value_angledegrees_string;
    std::string setting_key_temperature;
    Temperature setting_value_temperature = 0;    // Set initial value here since there is not default constructor
    std::string setting_value_temperature_string;
    std::string setting_key_velocity;
    Velocity setting_value_velocity = 0;    // Set initial value here since there is not default constructor
    std::string setting_value_velocity_string;
    std::string setting_key_velocity2;
    Velocity setting_value_velocity2 = 0;    // Set initial value here since there is not default constructor
    std::string setting_value_velocity_string2;
    std::string setting_key_ratio;
    Ratio setting_value_ratio = 0;    // Set initial value here since there is not default constructor
    std::string setting_value_ratio_string;
    std::string setting_key_duration;
    Duration setting_value_duration = 0;    // Set initial value here since there is not default constructor
    std::string setting_value_duration_string;
    std::string setting_key_duration2;
    Duration setting_value_duration2 = 0;    // Set initial value here since there is not default constructor
    std::string setting_value_duration_string2;
    std::string setting_key_flowtempgraph;
    double setting_value_flowtempgraph_flow;
    double setting_value_flowtempgraph_temp;
    double setting_value_flowtempgraph_flow2;
    double setting_value_flowtempgraph_temp2;
    double setting_value_flowtempgraph_flow3;
    double setting_value_flowtempgraph_temp3;
    std::string setting_value_flowtempgraph_string;
    std::string setting_key_fmatrix3x3;
    double setting_value_fmatrix3x3[3][3];
    std::string setting_value_fmatrix3x3_string;
    std::string setting_key_vector;
    std::vector<int> setting_value_vector;
    std::string setting_value_vector_string;
};

}

#endif // SETTINGS_TEST_H

