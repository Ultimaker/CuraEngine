//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.
#include "SettingsTest.h"

namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(SettingsTest);

void SettingsTest::setUp()
{
    my_settings = SettingsBase();
    my_settings.setSetting("intsetting", "123");
    my_settings.setSetting("intsetting0", "0");
    my_settings.setSetting("negintsetting", "-456");
    my_settings.setSetting("floatsetting", "123.456");
    my_settings.setSetting("negfloatsetting", "-456.123");
    my_settings.setSetting("strsetting", "hopsadee");
    my_settings.setSetting("boolsetting1a", "on");
    my_settings.setSetting("boolsetting1b", "off");
    my_settings.setSetting("boolsetting2a", "yes");
    my_settings.setSetting("boolsetting2b", "no");
    my_settings.setSetting("boolsetting3a", "true");
    my_settings.setSetting("boolsetting3b", "false");
}

void SettingsTest::tearDown()
{
}


void SettingsTest::smokeTest()
{
    my_settings.setSetting("bladibla", "my_value");
    my_settings.setSetting("setting2", "my_value2");
    CPPUNIT_ASSERT_MESSAGE("Different value expected1", my_settings.getSettingString("bladibla").compare("my_value") == 0);
    CPPUNIT_ASSERT_MESSAGE("Different value expected2", my_settings.getSettingString("setting2").compare("my_value2") == 0);
}

void SettingsTest::inheritBaseTest()
{
    SettingsBase child_settings = SettingsBase();
    child_settings.setSettingInheritBase("strsetting", my_settings);
    child_settings.setSetting("child_setting", "meeeh");
    CPPUNIT_ASSERT_MESSAGE("Different value expected", child_settings.getSettingString("strsetting").compare("hopsadee") == 0);
    CPPUNIT_ASSERT_MESSAGE("Different value expected", child_settings.getSettingString("child_setting").compare("meeeh") == 0);
}

void SettingsTest::getSettingTest()
{
    std::stringstream ss;
    CPPUNIT_ASSERT_MESSAGE("Get setting index failed", my_settings.getSettingAsIndex("intsetting") == 123);
    CPPUNIT_ASSERT_MESSAGE("Get setting index failed2", my_settings.getSettingAsIndex("negintsetting") == -456);
    CPPUNIT_ASSERT_MESSAGE("Get setting count failed", my_settings.getSettingAsIndex("intsetting") == 123);
    ss << "Get setting layer number failed: " << my_settings.getSettingAsLayerNumber("intsetting");
    CPPUNIT_ASSERT_MESSAGE(ss.str(), my_settings.getSettingAsLayerNumber("intsetting") == 123 - 1);
    // don't know why this doesn't work
//    ss << "Get setting layer number failed2: " << my_settings.getSettingAsLayerNumber("negintsetting");
//    CPPUNIT_ASSERT_MESSAGE("Get setting layer number failed2", my_settings.getSettingAsLayerNumber("negintsetting") == 0);
    ss << "Get setting layer number failed3: " << my_settings.getSettingAsLayerNumber("floatsetting");
    CPPUNIT_ASSERT_MESSAGE("Get setting layer number failed3", my_settings.getSettingAsLayerNumber("floatsetting") == 123 - 1);
}

void SettingsTest::getSettingTest2()
{
    CPPUNIT_ASSERT_MESSAGE("Get setting mm failed", std::abs(my_settings.getSettingInMillimeters("intsetting") - 123) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Get setting mm failed2", std::abs(my_settings.getSettingInMillimeters("floatsetting") - 123.456) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Get setting microns failed", std::abs(my_settings.getSettingInMicrons("intsetting") - 123000) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Get setting microns failed2", std::abs(my_settings.getSettingInMicrons("floatsetting") - 123456) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Get setting angle degrees failed", std::abs(my_settings.getSettingInAngleDegrees("intsetting") - 123) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Get setting angle radians failed", my_settings.getSettingInAngleRadians("intsetting") > 0);
}

void SettingsTest::getSettingTest3()
{
    CPPUNIT_ASSERT_MESSAGE("Get setting bool failed", my_settings.getSettingBoolean("intsetting") == true);
    CPPUNIT_ASSERT_MESSAGE("Get setting bool failed2", my_settings.getSettingBoolean("intsetting0") == false);

    CPPUNIT_ASSERT_MESSAGE("Get setting bool failed3", my_settings.getSettingBoolean("boolsetting1a") == true);
    CPPUNIT_ASSERT_MESSAGE("Get setting bool failed4", my_settings.getSettingBoolean("boolsetting1b") == false);
    CPPUNIT_ASSERT_MESSAGE("Get setting bool failed5", my_settings.getSettingBoolean("boolsetting2a") == true);
    CPPUNIT_ASSERT_MESSAGE("Get setting bool failed6", my_settings.getSettingBoolean("boolsetting2b") == false);
    CPPUNIT_ASSERT_MESSAGE("Get setting bool failed7", my_settings.getSettingBoolean("boolsetting3a") == true);
    CPPUNIT_ASSERT_MESSAGE("Get setting bool failed8", my_settings.getSettingBoolean("boolsetting3b") == false);
}

void SettingsTest::getSettingTest4()
{
    std::stringstream ss;
    CPPUNIT_ASSERT_MESSAGE("Get setting mm per s failed", std::abs(my_settings.getSettingInMillimetersPerSecond("floatsetting") - 123.456) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Get setting mm per s failed2", std::abs(my_settings.getSettingInMillimetersPerSecond("negfloatsetting") - 0) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Get setting mm3 failed", std::abs(my_settings.getSettingInCubicMillimeters("floatsetting") - 123.456) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Get setting mm3 failed2", std::abs(my_settings.getSettingInCubicMillimeters("negfloatsetting") - 0) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Get setting percentage failed", std::abs(my_settings.getSettingInPercentage("floatsetting") - 123.456) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Get setting percentage failed2", std::abs(my_settings.getSettingInPercentage("negfloatsetting") - 0) < epsilon);
    ss << "Get setting ratio failed" << my_settings.getSettingAsRatio("floatsetting");
    CPPUNIT_ASSERT_MESSAGE(ss.str(), std::abs(my_settings.getSettingAsRatio("floatsetting") - 1.23456) < epsilon);

    // Shouldn't the ratio behave like percentage?
    //CPPUNIT_ASSERT_MESSAGE("Get setting ratio failed2", std::abs(my_settings.getSettingAsRatio("negfloatsetting") - 0) < epsilon);

    CPPUNIT_ASSERT_MESSAGE("Get setting s failed", std::abs(my_settings.getSettingInSeconds("floatsetting") - 123.456) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Get setting s failed2", std::abs(my_settings.getSettingInSeconds("negfloatsetting") - 0) < epsilon);
}

void SettingsTest::getFlowTempGraphTest()
{
    std::stringstream ss;
    my_settings.setSetting("material_flow_temp_graph", "[[3.5,200],[7.0,240]]");
    FlowTempGraph flow_temp_graph = my_settings.getSettingAsFlowTempGraph("material_flow_temp_graph");
    CPPUNIT_ASSERT_MESSAGE("Get flow temp graph failed", std::abs(flow_temp_graph.getTemp(3.5, 190, true) - 200.0) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Get flow temp graph failed2", std::abs(flow_temp_graph.getTemp(3.5, 190, false) - 190.0) < epsilon);

    CPPUNIT_ASSERT_MESSAGE("Get flow temp graph failed3", std::abs(flow_temp_graph.getTemp(7, 190, true) - 240.0) < epsilon);
    ss << "Get flow temp graph failed4" << flow_temp_graph.getTemp(5.25, 190, true);
    CPPUNIT_ASSERT_MESSAGE(ss.str(), std::abs(flow_temp_graph.getTemp(5.25, 190, true) - 220.0) < epsilon);

    CPPUNIT_ASSERT_MESSAGE("Get flow temp graph failed5", std::abs(flow_temp_graph.getTemp(2, 190, true) - 200.0) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Get flow temp graph failed6", std::abs(flow_temp_graph.getTemp(12, 190, true) - 240.0) < epsilon);
}

void SettingsTest::getFlowTempGraphTest2()
{
    std::stringstream ss;
    my_settings.setSetting("error_flow_temp_graph", "[[3.5,200,[7.0,240],[3.0], [1.5, 180]]");
    // It should crash or give an error or so, not ignoring stuff and take some random implementation dependent values.
    // Taking default value may be the wanted action
    FlowTempGraph flow_temp_graph = my_settings.getSettingAsFlowTempGraph("error_flow_temp_graph");
    ss << "Get error flow temp graph failed: " << flow_temp_graph.getTemp(3.5, 190, true);
//    CPPUNIT_ASSERT_MESSAGE(ss.str(), std::abs(flow_temp_graph.getTemp(3.5, 190, true) - 190.0) < epsilon);
}

void SettingsTest::getMatrixTest()
{
    my_settings.setSetting("gimme_matrix", "[[0,1.5,0], [0,2,0], [0,0,2]]");
    FMatrix3x3 mat = my_settings.getSettingAsPointMatrix("gimme_matrix");
    CPPUNIT_ASSERT_MESSAGE("Expected matrix1", std::abs(mat.m[0][0] - 0) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Expected matrix2", std::abs(mat.m[1][0] - 1.5) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Expected matrix3", std::abs(mat.m[2][0] - 0) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Expected matrix4", std::abs(mat.m[0][1] - 0) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Expected matrix5", std::abs(mat.m[1][1] - 2) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Expected matrix6", std::abs(mat.m[2][1] - 0) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Expected matrix7", std::abs(mat.m[0][2] - 0) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Expected matrix8", std::abs(mat.m[1][2] - 0) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Expected matrix9", std::abs(mat.m[2][2] - 2) < epsilon);
}

void SettingsTest::getMatrixTest2()
{
    my_settings.setSetting("gimme_matrix", "[[0,1.5,0], [0,2,0], 0,0,2]]");  // error matrix
    FMatrix3x3 mat = my_settings.getSettingAsPointMatrix("gimme_matrix");
    // what should be the behaviour?
}

void SettingsTest::getIntegerListTest()
{
    my_settings.setSetting("intlist", "[15,42,0,2,2]");
    std::vector<int> vec = my_settings.getSettingAsIntegerList("intlist");
    CPPUNIT_ASSERT_MESSAGE("Expected int list1", std::abs(vec[0] - 15) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Expected int list2", std::abs(vec[1] - 42) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Expected int list3", std::abs(vec[2] - 0) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Expected int list4", std::abs(vec[3] - 2) < epsilon);
    CPPUNIT_ASSERT_MESSAGE("Expected int list5", std::abs(vec[4] - 2) < epsilon);
}

void SettingsTest::getIntegerListTest2()  // now with error input
{
    my_settings.setSetting("intlist", "[a15,42,0,2,2");
    std::vector<int> vec = my_settings.getSettingAsIntegerList("intlist");
//    CPPUNIT_ASSERT_MESSAGE("Expected int list1", std::abs(vec[0] - 15) < epsilon);
//    CPPUNIT_ASSERT_MESSAGE("Expected int list2", std::abs(vec[1] - 42) < epsilon);
//    CPPUNIT_ASSERT_MESSAGE("Expected int list3", std::abs(vec[2] - 0) < epsilon);
//    CPPUNIT_ASSERT_MESSAGE("Expected int list4", std::abs(vec[3] - 2) < epsilon);
//    CPPUNIT_ASSERT_MESSAGE("Expected int list5", std::abs(vec[4] - 2) < epsilon);
    // maybe we should just raise an error...
}

}  // namespace
