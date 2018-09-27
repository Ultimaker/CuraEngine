//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <cmath> //For M_PI.
#include <memory> //For shared_ptr.

#include "SettingsTest.h"
#include "../src/Application.h" //To test extruder train settings.
#include "../src/settings/FlowTempGraph.h"
#include "../src/settings/types/LayerIndex.h"
#include "../src/settings/types/AngleRadians.h"
#include "../src/settings/types/AngleDegrees.h"
#include "../src/settings/types/Temperature.h"
#include "../src/settings/types/Velocity.h"
#include "../src/settings/types/Ratio.h"
#include "../src/settings/types/Duration.h"
#include "../src/utils/floatpoint.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(SettingsTest);

void SettingsTest::setUp()
{
    settings = Settings(); //New instance to test with.
}

void SettingsTest::addSettingStringTest()
{
    const std::string setting_value("The human body contains enough bones to make an entire skeleton.");
    settings.add("test_setting", setting_value);
    CPPUNIT_ASSERT_EQUAL(setting_value, settings.get<std::string>("test_setting"));
}

void SettingsTest::addSettingDoubleTest()
{
    settings.add("test_setting", "1234567.890");
    CPPUNIT_ASSERT_EQUAL(double(1234567.89), settings.get<double>("test_setting"));
}

void SettingsTest::addSettingSizeTTest()
{
    settings.add("test_setting", "666");
    CPPUNIT_ASSERT_EQUAL(size_t(666), settings.get<size_t>("test_setting"));
}

void SettingsTest::addSettingBoolTest()
{
    settings.add("test_setting", "true");
    CPPUNIT_ASSERT_EQUAL(true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "on");
    CPPUNIT_ASSERT_EQUAL(true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "yes");
    CPPUNIT_ASSERT_EQUAL(true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "True");
    CPPUNIT_ASSERT_EQUAL(true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "50");
    CPPUNIT_ASSERT_EQUAL(true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "0");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("0 should cast to false.",
                                 false, settings.get<bool>("test_setting"));

    settings.add("test_setting", "false");
    CPPUNIT_ASSERT_EQUAL(false, settings.get<bool>("test_setting"));

    settings.add("test_setting", "False");
    CPPUNIT_ASSERT_EQUAL(false, settings.get<bool>("test_setting"));
}

class Slice; //Forward declaration to save some time compiling.

void SettingsTest::addSettingExtruderTrainTest()
{
    //Add a slice with some extruder trains.
    std::shared_ptr<Slice> current_slice = std::make_shared<Slice>(0);
    Application::getInstance().current_slice = current_slice.get();
    current_slice->scene.extruders.emplace_back(0, nullptr);
    current_slice->scene.extruders.emplace_back(1, nullptr);
    current_slice->scene.extruders.emplace_back(2, nullptr);

    settings.add("test_setting", "2");
    CPPUNIT_ASSERT_EQUAL(&current_slice->scene.extruders[2], &settings.get<ExtruderTrain&>("test_setting"));

    settings.add("extruder_nr", "1");
    settings.add("test_setting", "-1"); //-1 should let it fall back to the current extruder_nr.
    CPPUNIT_ASSERT_EQUAL_MESSAGE("If the extruder is negative, it uses the extruder_nr setting.",
                                 &current_slice->scene.extruders[1], &settings.get<ExtruderTrain&>("test_setting"));
}

void SettingsTest::addSettingLayerIndexTest()
{
    settings.add("test_setting", "4");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("LayerIndex settings start counting from 0, so subtract one.",
                                 LayerIndex(3), settings.get<LayerIndex>("test_setting"));
}

void SettingsTest::addSettingLayerIndexNegativeTest()
{
    settings.add("test_setting", "-10");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("LayerIndex settings still subtract 1 even in negative layers.",
                                 LayerIndex(-11), settings.get<LayerIndex>("test_setting"));
}

void SettingsTest::addSettingCoordTTest()
{
    settings.add("test_setting", "8589934.592"); //2^33 microns, so this MUST be a 64-bit integer! (Or at least 33-bit, but those don't exist.)
    CPPUNIT_ASSERT_EQUAL_MESSAGE("Coordinates must be entered in the setting as millimetres, but are converted to micrometres.",
                                 coord_t(8589934592), settings.get<coord_t>("test_setting"));
}

void SettingsTest::addSettingAngleRadiansTest()
{
    settings.add("test_setting", "180");
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("180 degrees is 1 pi radians.",
                                         AngleRadians(M_PI), settings.get<AngleRadians>("test_setting"), DELTA);

    settings.add("test_setting", "810");
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("810 degrees in clock arithmetic is 90 degrees, which is 0.5 pi radians.",
                                 AngleRadians(M_PI / 2.0), settings.get<AngleRadians>("test_setting"), DELTA);
}

void SettingsTest::addSettingAngleDegreesTest()
{
    settings.add("test_setting", "4442.4");
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("4442.4 in clock arithmetic is 360 degrees.",
                                         AngleDegrees(122.4), settings.get<AngleDegrees>("test_setting"), DELTA);
}

void SettingsTest::addSettingTemperatureTest()
{
    settings.add("test_setting", "245.5");
    CPPUNIT_ASSERT_DOUBLES_EQUAL(Temperature(245.5), settings.get<Temperature>("test_setting"), DELTA);
}

void SettingsTest::addSettingVelocityTest()
{
    settings.add("test_setting", "12.345");
    CPPUNIT_ASSERT_DOUBLES_EQUAL(Velocity(12.345), settings.get<Velocity>("test_setting"), DELTA);

    settings.add("test_setting", "-78");
    CPPUNIT_ASSERT_DOUBLES_EQUAL(Velocity(-78), settings.get<Velocity>("test_setting"), DELTA);
}

void SettingsTest::addSettingRatioTest()
{
    settings.add("test_setting", "1.618");
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("With ratios, the input is interpreted in percentages.",
                                         Ratio(0.01618), settings.get<Ratio>("test_setting"), DELTA);
}

void SettingsTest::addSettingDurationTest()
{
    settings.add("test_setting", "1234.5678");
    CPPUNIT_ASSERT_DOUBLES_EQUAL(Duration(1234.5678), settings.get<Duration>("test_setting"), DELTA);

    settings.add("test_setting", "-1234.5678");
    CPPUNIT_ASSERT_DOUBLES_EQUAL(Duration(0), settings.get<Duration>("test_setting"), DELTA);
}

void SettingsTest::addSettingFlowTempGraphTest()
{
    settings.add("test_setting", "[[1.50, 10.1],[ 25.1,40.4 ], [26.5,75], [50 , 100.10]]"); //Try various spacing and radixes.
    const FlowTempGraph flow_temp_graph = settings.get<FlowTempGraph>("test_setting");

    double stored_temperature = flow_temp_graph.getTemp(30.5, 200.0, true);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(75.0 + (100.10 - 75.0) * (30.5 - 26.5) / (50.0 - 26.5), stored_temperature, DELTA);

    stored_temperature = flow_temp_graph.getTemp(1, 200.0, true);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(10.1, stored_temperature, DELTA); //Flow too low - Return lower temperature in the graph.

    stored_temperature = flow_temp_graph.getTemp(80, 200.0, true);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(100.1, stored_temperature, DELTA); //Flow too high - Return higher temperature in the graph.

    stored_temperature = flow_temp_graph.getTemp(30.5, 200.0, false);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(200.0, stored_temperature, DELTA);
}

void SettingsTest::addSettingFMatrix3x3Test()
{
    settings.add("test_setting", "[[1.0, 2.0, 3.3],[ 2 , 3.0 , 1.0],[3.0 ,1.0,2.0 ]]"); //Try various spacing and radixes.
    FMatrix3x3 float_matrix = settings.get<FMatrix3x3>("test_setting");

    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, float_matrix.m[0][0], DELTA);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, float_matrix.m[1][0], DELTA);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3.3, float_matrix.m[2][0], DELTA);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, float_matrix.m[0][1], DELTA);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, float_matrix.m[1][1], DELTA);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, float_matrix.m[2][1], DELTA);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3.0, float_matrix.m[0][2], DELTA);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.0, float_matrix.m[1][2], DELTA);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, float_matrix.m[2][2], DELTA);
}

void SettingsTest::addSettingVectorTest()
{
    settings.add("test_setting", "[0, 1, 1,2, 3 , 5,  8,13]");
    std::vector<int> vector_int = settings.get<std::vector<int>>("test_setting");
    std::vector<int> ground_truth = {0, 1, 1, 2, 3, 5, 8, 13};
    CPPUNIT_ASSERT_EQUAL(ground_truth.size(), vector_int.size());
    for (size_t i = 0; i < ground_truth.size(); i++)
    {
        CPPUNIT_ASSERT_EQUAL(ground_truth[i], vector_int[i]);
    }
}

void SettingsTest::overwriteSettingTest()
{
    settings.add("test_setting", "P");
    settings.add("test_setting", "NP");
    CPPUNIT_ASSERT_MESSAGE("When overriding a setting, the original value was not changed.",
                           settings.get<std::string>("test_setting") != std::string("P"));
    CPPUNIT_ASSERT_EQUAL(std::string("NP"), settings.get<std::string>("test_setting"));
}

void SettingsTest::inheritanceTest()
{
    std::shared_ptr<Slice> current_slice = std::make_shared<Slice>(0);
    Application::getInstance().current_slice = current_slice.get();

    const std::string value = "A nuclear explosion would be a disaster.";
    Settings parent;
    parent.add("test_setting", value);
    settings.setParent(&parent);

    CPPUNIT_ASSERT_EQUAL(value, settings.get<std::string>("test_setting"));

    const std::string override_value = "It's quick, it's easy and it's free: Pouring river water in your socks.";
    settings.add("test_setting", override_value);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The new value overrides the one from the parent.",
                                 override_value, settings.get<std::string>("test_setting"));
}

void SettingsTest::limitToExtruderTest()
{
    std::shared_ptr<Slice> current_slice = std::make_shared<Slice>(0);
    Application::getInstance().current_slice = current_slice.get();
    current_slice->scene.extruders.emplace_back(0, nullptr);
    current_slice->scene.extruders.emplace_back(1, nullptr);
    current_slice->scene.extruders.emplace_back(2, nullptr);

    //Add a setting to the extruder this is limiting to.
    const std::string limit_extruder_value = "I was gonna tell a time travelling joke but you didn't like it.";
    current_slice->scene.extruders[2].settings.add("test_setting", limit_extruder_value);
    current_slice->scene.limit_to_extruder.emplace("test_setting", &current_slice->scene.extruders[2]);

    //Add a decoy setting to the main scene to make sure that we aren't getting the global setting instead.
    current_slice->scene.settings.add("test_setting", "Sting has been kidnapped. The Police have no lead.");

    CPPUNIT_ASSERT_EQUAL(limit_extruder_value, settings.get<std::string>("test_setting"));
}

}
