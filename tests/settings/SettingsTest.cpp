//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SettingsTest.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(SettingsTest);

void SettingsTest::setUp()
{
    setting_value_string = "setting_value_string";
    setting_value_int = 1;
    setting_value_int_string = "1";
    setting_value_int2 = -1;
    setting_value_int_string2 = "-1";
    setting_value_double = 1234567.890;
    setting_value_double_string = "1234567.890";
    setting_value_size_t = 1;
    setting_value_size_t_string = "1";
    setting_value_unsigned_int = 1;
    setting_value_unsigned_int_string = "1";
    setting_value_bool = true;
    setting_value_bool_string = "on";
    setting_value_bool2 = true;
    setting_value_bool_string2 = "yes";
    setting_value_bool3 = true;
    setting_value_bool_string3 = "True";
    setting_value_bool4 = true;
    setting_value_bool_string4 = "50";
    setting_value_bool5 = false;
    setting_value_bool_string5 = "0";
    setting_value_layerindex = 1;
    setting_value_layerindex_string = "1";
    setting_value_coord_t = 123456789;
    setting_value_coord_t_string = "123456.789";
    setting_value_angleradians = 225; // = 225 in degrees = (5/4)*pi in radians
    setting_value_angleradians_string = "2385"; // = 6*360+225 in degrees = (5/4)*pi in radians
    setting_value_angledegrees = 122.4; // =0.34*360
    setting_value_angledegrees_string = "4442.4"; // =12.34*360
    setting_value_temperature = 245.5;
    setting_value_temperature_string = "245.5";
    setting_value_velocity = 12.345;
    setting_value_velocity_string = "12.345";
    setting_value_velocity2 = 0;
    setting_value_velocity_string2 = "-12.345";
    setting_value_ratio = 123.4;
    setting_value_ratio_string = "123.4";
    setting_value_duration = 1234.5678;
    setting_value_duration_string = "1234.5678";
    setting_value_duration2 = 0;
    setting_value_duration_string2 = "-1234.5678";
    setting_value_flowtempgraph_flow = 30.5;
    setting_value_flowtempgraph_temp = 79.272340425531914893617021276596; // 75 + (100.10 - 75) * (30.5 - 26.5) / (50 - 26.5)
    setting_value_flowtempgraph_flow2 = 1;
    setting_value_flowtempgraph_temp2 = 10.1; // Flow too low - return lower temperature in the graph
    setting_value_flowtempgraph_flow3 = 80;
    setting_value_flowtempgraph_temp3 = 100.10; // Flow too high - return higher temperature in the graph
    setting_value_flowtempgraph_string = "[[1.50, 10.1],[ 25.1,40.4 ], [26.5,75], [50 , 100.10]]";
    setting_value_fmatrix3x3[0][0] = 1.0;
    setting_value_fmatrix3x3[0][1] = 2.0;
    setting_value_fmatrix3x3[0][2] = 3.0;
    setting_value_fmatrix3x3[1][0] = 2.0;
    setting_value_fmatrix3x3[1][1] = 3.0;
    setting_value_fmatrix3x3[1][2] = 1.0;
    setting_value_fmatrix3x3[2][0] = 3.0;
    setting_value_fmatrix3x3[2][1] = 1.0;
    setting_value_fmatrix3x3[2][2] = 2.0;
    setting_value_fmatrix3x3_string = "[[1.0, 2.0, 3.0],[ 2.0 , 3.0 , 1.0],[3.0 ,1.0,2.0 ]]";
    setting_value_vector = {0, 1, 2, 3, 5, 8, 13};
    setting_value_vector_string = "[0, 1,2, 3 ,  5, 8,13]";
}

void SettingsTest::addSettingStringTest()
{
    settings.add("test_setting", setting_value_string);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'string' setting is not the same as the expected value!",
                                 setting_value_string, settings.get<std::string>("test_setting"));
}

void SettingsTest::addSettingIntTest()
{
    settings.add("test_setting", setting_value_int_string);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'int' setting is not the same as the expected value!",
                                 setting_value_int, settings.get<int>("test_setting"));

    settings.add("test_setting", setting_value_int_string2);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'int' setting #2 is not the same as the expected value!",
                                 setting_value_int2, settings.get<int>("test_setting"));
}

void SettingsTest::addSettingDoubleTest()
{
    settings.add("test_setting", setting_value_double_string);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'double' setting is not the same as the expected value!",
                                 setting_value_double, settings.get<double>("test_setting"));
}

void SettingsTest::addSettingSizeTTest()
{
    settings.add("test_setting", setting_value_size_t_string);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'size_t' setting is not the same as the expected value!",
                                 setting_value_size_t, settings.get<size_t >("test_setting"));
}

void SettingsTest::addSettingUnsignedIntTest()
{
    settings.add("test_setting", setting_value_unsigned_int_string);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'unsigned int' setting is not the same as the expected value!",
                                 setting_value_unsigned_int, settings.get<unsigned int>("test_setting"));
}

void SettingsTest::addSettingBoolTest()
{
    settings.add("test_setting", setting_value_bool_string);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'bool' setting is not the same as the expected value!",
                                 setting_value_bool, settings.get<bool>("test_setting"));

    settings.add("test_setting", setting_value_bool_string2);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'bool' setting #2 is not the same as the expected value!",
                                 setting_value_bool2, settings.get<bool>("test_setting"));

    settings.add("test_setting", setting_value_bool_string3);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'bool' setting #3 is not the same as the expected value!",
                                 setting_value_bool3, settings.get<bool>("test_setting"));

    settings.add("test_setting", setting_value_bool_string4);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'bool' setting #4 is not the same as the expected value!",
                                 setting_value_bool4, settings.get<bool>("test_setting"));

    settings.add("test_setting", setting_value_bool_string5);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'bool' setting #5 is not the same as the expected value!",
                                 setting_value_bool5, settings.get<bool>("test_setting"));
}

void SettingsTest::addSettingExtruderTrainTest()
{
    // TODO: Do it when the implementation is done.
    CPPUNIT_ASSERT_MESSAGE("TODO: The value of the 'ExtruderTrain' setting is not the same as the expected value!",
                           false);
}

void SettingsTest::addSettingLayerIndexTest()
{
    settings.add("test_setting", setting_value_layerindex_string);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'LayerIndex' setting is not the same as the expected value!",
                                 setting_value_layerindex, settings.get<LayerIndex>("test_setting"));
}

void SettingsTest::addSettingCoordTTest()
{
    settings.add("test_setting", setting_value_coord_t_string);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'coord_t' setting is not the same as the expected value!",
                                 setting_value_coord_t, settings.get<coord_t>("test_setting"));
}

void SettingsTest::addSettingAngleRadiansTest()
{
    settings.add("test_setting", setting_value_angleradians_string);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'AngleRadians' setting is not the same as the expected value!",
                                         setting_value_angleradians, settings.get<AngleRadians>("test_setting"), DELTA);
}

void SettingsTest::addSettingAngleDegreesTest()
{
    settings.add("test_setting", setting_value_angledegrees_string);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'AngleDegrees' setting is not the same as the expected value!",
                                         setting_value_angledegrees, settings.get<AngleDegrees>("test_setting"), DELTA);
}

void SettingsTest::addSettingTemperatureTest()
{
    settings.add("test_setting", setting_value_temperature_string);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'Temperature' setting is not the same as the expected value!",
                                         setting_value_temperature, settings.get<Temperature>("test_setting"), DELTA);
}

void SettingsTest::addSettingVelocityTest()
{
    settings.add("test_setting", setting_value_velocity_string);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'Velocity' setting is not the same as the expected value!",
                                         setting_value_velocity, settings.get<Velocity>("test_setting"), DELTA);

    settings.add("test_setting", setting_value_velocity_string2);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'Velocity' setting #2 is not the same as the expected value!",
                                         setting_value_velocity2, settings.get<Velocity>("test_setting"), DELTA);
}

void SettingsTest::addSettingRatioTest()
{
    settings.add("test_setting", setting_value_ratio_string);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'Ratio' setting is not the same as the expected value!",
                                         setting_value_ratio, settings.get<Ratio>("test_setting"), DELTA);
}

void SettingsTest::addSettingDurationTest()
{
    settings.add("test_setting", setting_value_duration_string);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'Duration' setting is not the same as the expected value!",
                                         setting_value_duration, settings.get<Duration>("test_setting"), DELTA);

    settings.add("test_setting", setting_value_duration_string2);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'Duration' setting #2 is not the same as the expected value!",
                                         setting_value_duration2, settings.get<Duration>("test_setting"), DELTA);
}

void SettingsTest::addSettingFlowTempGraphTest()
{
    settings.add("test_setting", setting_value_flowtempgraph_string);
    FlowTempGraph flow_temp_graph = settings.get<FlowTempGraph>("test_setting");
    double material_print_temperature = 200.0;
    double stored_temperature = flow_temp_graph.getTemp(setting_value_flowtempgraph_flow, material_print_temperature, true);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'FlowTempGraph' setting for flow #1 is not the same as the expected value!",
                                         setting_value_flowtempgraph_temp, stored_temperature, DELTA);

    stored_temperature = flow_temp_graph.getTemp(setting_value_flowtempgraph_flow2, material_print_temperature, true);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'FlowTempGraph' setting for flow #2 is not the same as the expected value!",
                                         setting_value_flowtempgraph_temp2, stored_temperature, DELTA);

    stored_temperature = flow_temp_graph.getTemp(setting_value_flowtempgraph_flow3, material_print_temperature, true);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'FlowTempGraph' setting for flow #3 is not the same as the expected value!",
                                         setting_value_flowtempgraph_temp3, stored_temperature, DELTA);

    stored_temperature = flow_temp_graph.getTemp(setting_value_flowtempgraph_flow, material_print_temperature, false);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'FlowTempGraph' setting when not flow dependant temperature"
                                                 " is not the same as the expected value!",
                                         material_print_temperature, stored_temperature, DELTA);
}

void SettingsTest::addSettingFMatrix3x3Test()
{
    settings.add("test_setting", setting_value_fmatrix3x3_string);
    FMatrix3x3 flow_matrix = settings.get<FMatrix3x3>("test_setting");
    for (size_t x = 0; x < 3; x++)
    {
        for (size_t y = 0; y < 3; y++)
        {
            CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'FMatrix3x3' setting is not the same as the expected value!",
                                                 setting_value_fmatrix3x3[x][y], flow_matrix.m[x][y], DELTA);
        }
    }
}

void SettingsTest::addSettingVectorTest()
{
    settings.add("test_setting", setting_value_vector_string);
    std::vector<int> vector_int = settings.get<std::vector<int>>("test_setting");
    for (size_t i = 0; i < setting_value_vector.size(); i++)
    {
        CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'vector<int>' setting is not the same as the expected value!",
                                             setting_value_vector[i], vector_int[i]);
    }

}

void SettingsTest::overwriteSettingTest()
{
    settings.add("test_setting", setting_value_int_string);
    settings.add("test_setting", setting_value_int_string2);
    CPPUNIT_ASSERT_MESSAGE("The two values to be inserted must be different!",
                           setting_value_int_string != setting_value_int_string2);
    CPPUNIT_ASSERT_MESSAGE("The value of the 'int' setting must not be equals to the second value inserted!",
                           settings.get<int>("test_setting") != setting_value_int2);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'int' setting must be equals to the first value inserted!",
                                 setting_value_int, settings.get<int>("test_setting"));
}

}
