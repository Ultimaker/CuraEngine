//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SETTINGS_TEST_H
#define SETTINGS_TEST_H

#include <functional> // function

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "../src/settings/Setting.h"
#include "../src/settings/Settings.h"

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
     * \brief Test a setting with a string value is correctly inserted
     */
    void addSettingStringTest();

    /*!
     * \brief Test a setting with an int value is correctly inserted
     */
    void addSettingIntTest();

    /*!
     * \brief Test a setting with a double value is correctly inserted
     */
    void addSettingDoubleTest();

    /*!
     * \brief Test a setting with a size_t value is correctly inserted
     */
    void addSettingSizeTTest();

    /*!
     * \brief Test a setting with an unsigned int value is correctly inserted
     */
    void addSettingUnsignedIntTest();

    /*!
     * \brief Test a setting with an bool value is correctly inserted
     */
    void addSettingBoolTest();

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
};

}

#endif // SETTINGS_TEST_H

