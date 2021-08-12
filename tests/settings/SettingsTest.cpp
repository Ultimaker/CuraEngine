//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <cmath> //For M_PI.
#include <gtest/gtest.h>
#include <memory> //For shared_ptr.

#include "../src/Application.h" //To test extruder train settings.
#include "../src/ExtruderTrain.h"
#include "../src/Slice.h"
#include "../src/settings/FlowTempGraph.h"
#include "../src/settings/Settings.h" //The class under test.
#include "../src/settings/types/LayerIndex.h"
#include "../src/settings/types/Angle.h"
#include "../src/settings/types/Temperature.h"
#include "../src/settings/types/Velocity.h"
#include "../src/settings/types/Ratio.h"
#include "../src/settings/types/Duration.h"
#include "../src/utils/FMatrix4x3.h" //Testing matrix transformation settings.

namespace cura
{

/*
 * A test fixture with an empty settings object to test with.
 */
class SettingsTest : public testing::Test
{
public:
    Settings settings;
};

TEST_F(SettingsTest, AddSettingString)
{
    const std::string setting_value("The human body contains enough bones to make an entire skeleton.");
    settings.add("test_setting", setting_value);
    EXPECT_EQ(setting_value, settings.get<std::string>("test_setting"));
}

TEST_F(SettingsTest, AddSettingDouble)
{
    settings.add("test_setting", "1234567.890");
    EXPECT_DOUBLE_EQ(double(1234567.89), settings.get<double>("test_setting"));
}

TEST_F(SettingsTest, AddSettingSizeT)
{
    settings.add("test_setting", "666");
    EXPECT_EQ(size_t(666), settings.get<size_t>("test_setting"));
}

TEST_F(SettingsTest, AddSettingBool)
{
    settings.add("test_setting", "true");
    EXPECT_EQ(true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "on");
    EXPECT_EQ(true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "yes");
    EXPECT_EQ(true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "True");
    EXPECT_EQ(true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "50");
    EXPECT_EQ(true, settings.get<bool>("test_setting"));

    settings.add("test_setting", "0");
    EXPECT_EQ(false, settings.get<bool>("test_setting")) << "0 should cast to false.";

    settings.add("test_setting", "false");
    EXPECT_EQ(false, settings.get<bool>("test_setting"));

    settings.add("test_setting", "False");
    EXPECT_EQ(false, settings.get<bool>("test_setting"));
}

class Slice; //Forward declaration to save some time compiling.

TEST_F(SettingsTest, AddSettingExtruderTrain)
{
    //Add a slice with some extruder trains.
    std::shared_ptr<Slice> current_slice = std::make_shared<Slice>(0);
    Application::getInstance().current_slice = current_slice.get();
    current_slice->scene.extruders.emplace_back(0, nullptr);
    current_slice->scene.extruders.emplace_back(1, nullptr);
    current_slice->scene.extruders.emplace_back(2, nullptr);

    settings.add("test_setting", "2");
    EXPECT_EQ(&current_slice->scene.extruders[2], &settings.get<ExtruderTrain&>("test_setting"));

    settings.add("extruder_nr", "1");
    settings.add("test_setting", "-1"); //-1 should let it fall back to the current extruder_nr.
    EXPECT_EQ(&current_slice->scene.extruders[1], &settings.get<ExtruderTrain&>("test_setting")) << "If the extruder is negative, it uses the extruder_nr setting.";
}

TEST_F(SettingsTest, AddSettingLayerIndex)
{
    settings.add("test_setting", "4");
    EXPECT_EQ(LayerIndex(3), settings.get<LayerIndex>("test_setting")) << "LayerIndex settings start counting from 0, so subtract one.";
}

TEST_F(SettingsTest, AddSettingLayerIndexNegative)
{
    settings.add("test_setting", "-10");
    EXPECT_EQ(LayerIndex(-11), settings.get<LayerIndex>("test_setting")) << "LayerIndex settings still subtract 1 even in negative layers.";
}

TEST_F(SettingsTest, AddSettingCoordT)
{
    settings.add("test_setting", "8589934.592"); //2^33 microns, so this MUST be a 64-bit integer! (Or at least 33-bit, but those don't exist.)
    EXPECT_EQ(coord_t(8589934592), settings.get<coord_t>("test_setting")) << "Coordinates must be entered in the setting as millimetres, but are converted to micrometres.";
}

TEST_F(SettingsTest, AddSettingAngleRadians)
{
    settings.add("test_setting", "180");
    EXPECT_DOUBLE_EQ(AngleRadians(M_PI), settings.get<AngleRadians>("test_setting")) << "180 degrees is 1 pi radians.";

    settings.add("test_setting", "810");
    EXPECT_NEAR(AngleRadians(M_PI / 2.0), settings.get<AngleRadians>("test_setting"), 0.00000001) << "810 degrees in clock arithmetic is 90 degrees, which is 0.5 pi radians.";
}

TEST_F(SettingsTest, AddSettingAngleDegrees)
{
    settings.add("test_setting", "4442.4");
    EXPECT_NEAR(AngleDegrees(122.4), settings.get<AngleDegrees>("test_setting"), 0.00000001) << "4320 is divisible by 360, so 4442.4 in clock arithmetic is 122.4 degrees.";
}

TEST_F(SettingsTest, AddSettingTemperature)
{
    settings.add("test_setting", "245.5");
    EXPECT_DOUBLE_EQ(Temperature(245.5), settings.get<Temperature>("test_setting"));
}

TEST_F(SettingsTest, AddSettingVelocity)
{
    settings.add("test_setting", "12.345");
    EXPECT_DOUBLE_EQ(Velocity(12.345), settings.get<Velocity>("test_setting"));

    settings.add("test_setting", "-78");
    EXPECT_DOUBLE_EQ(Velocity(-78), settings.get<Velocity>("test_setting"));
}

TEST_F(SettingsTest, AddSettingRatio)
{
    settings.add("test_setting", "1.618");
    EXPECT_DOUBLE_EQ(Ratio(0.01618), settings.get<Ratio>("test_setting")) << "With ratios, the input is interpreted in percentages.";
}

TEST_F(SettingsTest, AddSettingDuration)
{
    settings.add("test_setting", "1234.5678");
    EXPECT_DOUBLE_EQ(Duration(1234.5678), settings.get<Duration>("test_setting"));

    settings.add("test_setting", "-1234.5678");
    EXPECT_DOUBLE_EQ(Duration(0), settings.get<Duration>("test_setting")) << "Negative duration doesn't exist, so it gets rounded to 0.";
}

TEST_F(SettingsTest, AddSettingFlowTempGraph)
{
    settings.add("test_setting", "[[1.50, 10.1],[ 25.1,40.4 ], [26.5,75], [50 , 100.10]]"); //Try various spacing and radixes.
    const FlowTempGraph flow_temp_graph = settings.get<FlowTempGraph>("test_setting");

    double stored_temperature = flow_temp_graph.getTemp(30.5, 200.0, true);
    EXPECT_DOUBLE_EQ(75.0 + (100.10 - 75.0) * (30.5 - 26.5) / (50.0 - 26.5), stored_temperature) << "Interpolate between low and high value.";

    stored_temperature = flow_temp_graph.getTemp(1, 200.0, true);
    EXPECT_DOUBLE_EQ(10.1, stored_temperature) << "Flow too low - Return lower temperature in the graph.";

    stored_temperature = flow_temp_graph.getTemp(80, 200.0, true);
    EXPECT_DOUBLE_EQ(100.1, stored_temperature) << "Flow too high - Return higher temperature in the graph.";

    stored_temperature = flow_temp_graph.getTemp(30.5, 200.0, false);
    EXPECT_DOUBLE_EQ(200.0, stored_temperature);
}

TEST_F(SettingsTest, AddSettingFMatrix3x3)
{
    settings.add("test_setting", "[[1.0, 2.0, 3.3],[ 2 , 3.0 , 1.0],[3.0 ,1.0,2.0 ]]"); //Try various spacing and radixes.
    FMatrix4x3 float_matrix = settings.get<FMatrix4x3>("test_setting");

    EXPECT_DOUBLE_EQ(1.0, float_matrix.m[0][0]);
    EXPECT_DOUBLE_EQ(2.0, float_matrix.m[1][0]);
    EXPECT_DOUBLE_EQ(3.3, float_matrix.m[2][0]);
    EXPECT_DOUBLE_EQ(2.0, float_matrix.m[0][1]);
    EXPECT_DOUBLE_EQ(3.0, float_matrix.m[1][1]);
    EXPECT_DOUBLE_EQ(1.0, float_matrix.m[2][1]);
    EXPECT_DOUBLE_EQ(3.0, float_matrix.m[0][2]);
    EXPECT_DOUBLE_EQ(1.0, float_matrix.m[1][2]);
    EXPECT_DOUBLE_EQ(2.0, float_matrix.m[2][2]);
}

TEST_F(SettingsTest, AddSettingVector)
{
    settings.add("test_setting", "[0, 1, 1,2, 3 , 5,  8,13]");
    const std::vector<int> vector_int = settings.get<std::vector<int>>("test_setting");
    const std::vector<int> ground_truth = {0, 1, 1, 2, 3, 5, 8, 13};
    ASSERT_EQ(ground_truth.size(), vector_int.size());
    for (size_t i = 0; i < ground_truth.size(); i++)
    {
        EXPECT_EQ(ground_truth[i], vector_int[i]);
    }
}

TEST_F(SettingsTest, OverwriteSetting)
{
    settings.add("test_setting", "P");
    settings.add("test_setting", "NP");
    ASSERT_NE(settings.get<std::string>("test_setting"), std::string("P")) << "When overriding a setting, the value must be changed.";
    ASSERT_EQ(settings.get<std::string>("test_setting"), std::string("NP"));
}

TEST_F(SettingsTest, Inheritance)
{
    std::shared_ptr<Slice> current_slice = std::make_shared<Slice>(0);
    Application::getInstance().current_slice = current_slice.get();

    const std::string value = "To be frank, I'd have to change my name.";
    Settings parent;
    parent.add("test_setting", value);
    settings.setParent(&parent);

    EXPECT_EQ(value, settings.get<std::string>("test_setting"));

    const std::string override_value = "It's quick, it's easy and it's free: Pouring river water in your socks.";
    settings.add("test_setting", override_value);
    EXPECT_EQ(override_value, settings.get<std::string>("test_setting")) << "The new value overrides the one from the parent.";
}

TEST_F(SettingsTest, LimitToExtruder)
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

    EXPECT_EQ(limit_extruder_value, settings.get<std::string>("test_setting"));
}

}
