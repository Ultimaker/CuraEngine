//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "GCodePlannerTest.h"

#include "../src/MeshGroup.h" //Needed to construct the GCodePlanner.

namespace cura
{

CPPUNIT_TEST_SUITE_REGISTRATION(GCodePlannerTest);

void GCodePlannerTest::setUp()
{
    storage = new SliceDataStorage(nullptr); //Empty data.
    FanSpeedLayerTimeSettings fan_speed_layer_time_settings; //A dummy fan speed and layer time settings.
    fan_speed_layer_time_settings.cool_min_layer_time = 0;
    fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max = 1;
    fan_speed_layer_time_settings.cool_fan_speed_min = 0;
    fan_speed_layer_time_settings.cool_fan_speed_max = 1;
    fan_speed_layer_time_settings.cool_min_speed = 0.5;
    //                              Command  Slice     layer  z  layer   last        current   fan speed and layer            retraction  comb    travel  travel avoid
    //                              socket   storage   nr        height  position    extruder  time settings                  combing     offset  avoid   distance
    gCodePlanner = new GCodePlanner(nullptr, *storage, 0,     0, 0.1,    Point(0,0), 0,        fan_speed_layer_time_settings, false,      100,    false,  50          );
}

void GCodePlannerTest::tearDown()
{
    delete gCodePlanner;
    gCodePlanner = nullptr;
    delete storage;
    storage = nullptr;
}

void GCodePlannerTest::computeNaiveTimeEstimatesRetractionTest()
{
    TimeMaterialEstimates estimate_empty = gCodePlanner->computeNaiveTimeEstimates(); //First try estimating time and material without any content.
    TimeMaterialEstimates estimate_empty_expected(0,0,0,0); //We expect the estimate of all time and material used to be 0.
    verifyEstimates(estimate_empty,estimate_empty_expected,"Empty GCodePlanner");
    
    GCodeExport gcode;
    GCodePathConfig configuration = storage->travel_config;
    gCodePlanner->addExtrusionMove(Point(0,0),&configuration,1.0f); //Need to have at least one path to have a configuration.
    TimeMaterialEstimates before_retract = gCodePlanner->computeNaiveTimeEstimates();
    gCodePlanner->writeRetraction(gcode,(unsigned int)0,(unsigned int)0); //Make a retract.
    TimeMaterialEstimates after_retract = gCodePlanner->computeNaiveTimeEstimates();
    TimeMaterialEstimates estimate_one_retraction = after_retract - before_retract;
    TimeMaterialEstimates estimate_one_retraction_expected(0,0,0,0);
    verifyEstimates(estimate_one_retraction,estimate_one_retraction_expected,"One retraction");
}

void GCodePlannerTest::verifyEstimates(const TimeMaterialEstimates& observed,const TimeMaterialEstimates& expected,std::string test_description)
{
    //Check each of the four estimates in the TimeMaterialEstimate instances.
    {
        std::stringstream ss;
        ss << test_description << ": Extrude time is " << observed.getExtrudeTime() << " instead of the expected " << expected.getExtrudeTime();
        CPPUNIT_ASSERT_MESSAGE(ss.str(),observed.getExtrudeTime() == expected.getExtrudeTime());
    }
    {
        std::stringstream ss;
        ss << test_description << ": Unretracted travel time is " << (observed.getTotalUnretractedTime() - observed.getExtrudeTime()) << " instead of the expected " << (expected.getTotalUnretractedTime() - expected.getExtrudeTime());
        CPPUNIT_ASSERT_MESSAGE(ss.str(),observed.getTotalUnretractedTime() - observed.getExtrudeTime() == expected.getTotalUnretractedTime() - expected.getExtrudeTime());
    }
    {
        std::stringstream ss;
        ss << test_description << ": Retracted travel time is " << (observed.getTotalTime() - observed.getTotalUnretractedTime()) << " instead of the expected " << (expected.getTotalTime() - expected.getTotalUnretractedTime());
        CPPUNIT_ASSERT_MESSAGE(ss.str(),observed.getTotalTime() - observed.getTotalUnretractedTime() == expected.getTotalTime() - expected.getTotalUnretractedTime());
    }
    {
        std::stringstream ss;
        ss << test_description << ": Material used is " << observed.getMaterial() << " instead of the expected " << expected.getMaterial();
        CPPUNIT_ASSERT_MESSAGE(ss.str(),observed.getMaterial() == expected.getMaterial());
    }
}

}