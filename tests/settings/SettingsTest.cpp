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
    setting_value_double = 1234567.89;
    setting_value_double_string = "1234567.89";
    setting_key_size_t = "setting_key_size_t";
    setting_value_size_t = 1;
    setting_value_size_t_string = "1";
    setting_key_unsigned_int = "setting_key_unsigned_int";
    setting_value_unsigned_int = 1;
    setting_value_unsigned_int_string = "1";
}

void SettingsTest::tearDown()
{
    //Do nothing.
}

void SettingsTest::addSettingStringTest()
{
    settings.add(setting_key_string, setting_value_string);
    CPPUNIT_ASSERT_MESSAGE("The value of the 'string' setting is not the same as the introduced value!",
                           settings.get<std::string>(setting_key_string) == setting_value_string);
}

void SettingsTest::addSettingIntTest()
{
    settings.add(setting_key_int, setting_value_int_string);
    CPPUNIT_ASSERT_MESSAGE("The value of the 'int' setting is not the same as the introduced value!",
                           settings.get<int>(setting_key_int) == setting_value_int);

    settings.add(setting_key_int2, setting_value_int_string2);
    CPPUNIT_ASSERT_MESSAGE("The value of the negative 'int' setting is not the same as the introduced value!",
                           settings.get<int>(setting_key_int2) == setting_value_int2);
}

void SettingsTest::addSettingDoubleTest()
{
    settings.add(setting_key_double, setting_value_double_string);
    CPPUNIT_ASSERT_MESSAGE("The value of the 'double' setting is not the same as the introduced value!",
                           settings.get<double>(setting_key_double) == setting_value_double);
}

void SettingsTest::addSettingSizeTTest()
{
    settings.add(setting_key_size_t, setting_value_size_t_string);
    CPPUNIT_ASSERT_MESSAGE("The value of the 'size_t' setting is not the same as the introduced value!",
                           settings.get<size_t >(setting_key_size_t) == setting_value_size_t);
}

void SettingsTest::addSettingUnsignedIntTest()
{
    settings.add(setting_key_unsigned_int, setting_value_unsigned_int_string);
    CPPUNIT_ASSERT_MESSAGE("The value of the 'double' setting is not the same as the introduced value!",
                           settings.get<unsigned int>(setting_key_unsigned_int) == setting_value_unsigned_int);
}

void SettingsTest::overwriteSettingTest()
{
    settings.add(setting_key_int, setting_value_int_string);
    settings.add(setting_key_int, setting_value_int_string2);
    CPPUNIT_ASSERT_MESSAGE("The two values to be inserted must be different!",
                           setting_value_int_string != setting_value_int_string2);
    CPPUNIT_ASSERT_MESSAGE("The value of the 'int' setting must not be equals to the second value inserted!",
                           settings.get<int>(setting_key_int) != setting_value_int2);
    CPPUNIT_ASSERT_MESSAGE("The value of the 'int' setting must be equals to the first value inserted!",
                           settings.get<int>(setting_key_int) == setting_value_int);
}

}
