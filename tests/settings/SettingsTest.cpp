//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SettingsTest.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(SettingsTest);

void SettingsTest::setUp()
{
    setting_key_string = "setting_key_string";
    setting_value_string = "setting_value_string";
    setting_key_int = "setting_key_int";
    setting_value_int = 1;
    setting_value_int_string = "1";
    setting_key_int2 = "setting_key_int2";
    setting_value_int2 = -1;
    setting_value_int_string2 = "-1";
    setting_key_double = "setting_key_double";
    setting_value_double = 1234567.890;
    setting_value_double_string = "1234567.890";
    setting_key_size_t = "setting_key_size_t";
    setting_value_size_t = 1;
    setting_value_size_t_string = "1";
    setting_key_unsigned_int = "setting_key_unsigned_int";
    setting_value_unsigned_int = 1;
    setting_value_unsigned_int_string = "1";
    setting_key_bool = "setting_key_bool";
    setting_value_bool = true;
    setting_value_bool_string = "on";
    setting_key_bool2 = "setting_key_bool2";
    setting_value_bool2 = true;
    setting_value_bool_string2 = "yes";
    setting_key_bool3 = "setting_key_bool3";
    setting_value_bool3 = true;
    setting_value_bool_string3 = "True";
    setting_key_bool4 = "setting_key_bool4";
    setting_value_bool4 = true;
    setting_value_bool_string4 = "50";
    setting_key_bool5 = "setting_key_bool5";
    setting_value_bool5 = false;
    setting_value_bool_string5 = "0";
    setting_key_layerindex = "setting_key_layerindex";
    setting_value_layerindex = 1;
    setting_value_layerindex_string = "1";
    setting_key_coord_t = "setting_key_size_t";
    setting_value_coord_t = 123456789;
    setting_value_coord_t_string = "123456.789";
    setting_key_angleradians = "setting_key_angleradians";
    setting_value_angleradians = 225; // = 225 in degrees = (5/4)*pi in radians
    setting_value_angleradians_string = "2385"; // = 6*360+225 in degrees = (5/4)*pi in radians
    setting_key_angledegrees = "setting_key_angledegrees";
    setting_value_angledegrees = 122.4; // =0.34*360
    setting_value_angledegrees_string = "4442.4"; // =12.34*360
}

void SettingsTest::tearDown()
{
    //Do nothing.
}

void SettingsTest::addSettingStringTest()
{
    settings.add(setting_key_string, setting_value_string);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'string' setting is not the same as the expected value!",
                                 setting_value_string, settings.get<std::string>(setting_key_string));
}

void SettingsTest::addSettingIntTest()
{
    settings.add(setting_key_int, setting_value_int_string);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'int' setting is not the same as the expected value!",
                                 setting_value_int, settings.get<int>(setting_key_int));

    settings.add(setting_key_int2, setting_value_int_string2);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'int' setting #2 is not the same as the expected value!",
                                 setting_value_int2, settings.get<int>(setting_key_int2));
}

void SettingsTest::addSettingDoubleTest()
{
    settings.add(setting_key_double, setting_value_double_string);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'double' setting is not the same as the expected value!",
                                 setting_value_double, settings.get<double>(setting_key_double));
}

void SettingsTest::addSettingSizeTTest()
{
    settings.add(setting_key_size_t, setting_value_size_t_string);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'size_t' setting is not the same as the expected value!",
                                 setting_value_size_t, settings.get<size_t >(setting_key_size_t));
}

void SettingsTest::addSettingUnsignedIntTest()
{
    settings.add(setting_key_unsigned_int, setting_value_unsigned_int_string);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'unsigned int' setting is not the same as the expected value!",
                                 setting_value_unsigned_int, settings.get<unsigned int>(setting_key_unsigned_int));
}

void SettingsTest::addSettingBoolTest()
{
    settings.add(setting_key_bool, setting_value_bool_string);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'bool' setting is not the same as the expected value!",
                                 setting_value_bool, settings.get<bool>(setting_key_bool));

    settings.add(setting_key_bool2, setting_value_bool_string2);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'bool' setting #2 is not the same as the expected value!",
                                 setting_value_bool2, settings.get<bool>(setting_key_bool2));

    settings.add(setting_key_bool3, setting_value_bool_string3);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'bool' setting #3 is not the same as the expected value!",
                                 setting_value_bool3, settings.get<bool>(setting_key_bool3));

    settings.add(setting_key_bool4, setting_value_bool_string4);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'bool' setting #4 is not the same as the expected value!",
                                 setting_value_bool4, settings.get<bool>(setting_key_bool4));

    settings.add(setting_key_bool5, setting_value_bool_string5);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'bool' setting #5 is not the same as the expected value!",
                                 setting_value_bool5, settings.get<bool>(setting_key_bool5));
}

void SettingsTest::addSettingExtruderTrainTest()
{
    // TODO: Do it when the implementation is done.
    CPPUNIT_ASSERT_MESSAGE("TODO: The value of the 'ExtruderTrain' setting is not the same as the expected value!",
                           false);
}

void SettingsTest::addSettingLayerIndexTest()
{
    settings.add(setting_key_layerindex, setting_value_layerindex_string);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'LayerIndex' setting is not the same as the expected value!",
                                 setting_value_layerindex, settings.get<LayerIndex>(setting_key_layerindex));
}

void SettingsTest::addSettingCoordTTest()
{
    settings.add(setting_key_coord_t, setting_value_coord_t_string);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'coord_t' setting is not the same as the expected value!",
                                 setting_value_coord_t, settings.get<coord_t>(setting_key_coord_t));
}

void SettingsTest::addSettingAngleRadiansTest()
{
    settings.add(setting_key_angleradians, setting_value_angleradians_string);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'AngleRadians' setting is not the same as the expected value!",
                                         setting_value_angleradians, settings.get<AngleRadians>(setting_key_angleradians), DELTA);
}

void SettingsTest::addSettingAngleDegreesTest()
{
    settings.add(setting_key_angledegrees, setting_value_angledegrees_string);
    CPPUNIT_ASSERT_DOUBLES_EQUAL_MESSAGE("The value of the 'AngleDegrees' setting is not the same as the expected value!",
                                         setting_value_angledegrees, settings.get<AngleDegrees>(setting_key_angledegrees), DELTA);
}

void SettingsTest::overwriteSettingTest()
{
    settings.add(setting_key_int, setting_value_int_string);
    settings.add(setting_key_int, setting_value_int_string2);
    CPPUNIT_ASSERT_MESSAGE("The two values to be inserted must be different!",
                           setting_value_int_string != setting_value_int_string2);
    CPPUNIT_ASSERT_MESSAGE("The value of the 'int' setting must not be equals to the second value inserted!",
                           settings.get<int>(setting_key_int) != setting_value_int2);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The value of the 'int' setting must be equals to the first value inserted!",
                                 setting_value_int, settings.get<int>(setting_key_int));
}

}
