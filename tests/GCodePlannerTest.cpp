//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "GCodePlannerTest.h"

#include "../src/MeshGroup.h" //Needed to construct the GCodePlanner.

#define ALLOWED_ESTIMATE_ERROR 0.1 //Fraction of the time estimates that the estimate is allowed to be off from the ground truth.

namespace cura
{

CPPUNIT_TEST_SUITE_REGISTRATION(GCodePlannerTest);

void GCodePlannerTest::setUp()
{
    SettingsBase settings;
    settings.setSetting("machine_extruder_count", "1");
    MeshGroup meshgroup(&settings);
    meshgroup.createExtruderTrain(0);

    storage = new SliceDataStorage(&meshgroup); //Empty data.
    storage->retraction_config_per_extruder[0].speed = 25; // set some semi realistic data
    storage->retraction_config_per_extruder[0].primeSpeed = 25;
    storage->retraction_config_per_extruder[0].distance = 10;

    // make a new GCodePathConfig and put it at a dummy place (note that the config is not an actual travel config!)
    storage->travel_config_per_extruder.emplace_back(PrintFeatureType::MoveCombing);
    storage->travel_config_per_extruder.back().init(60, MM2INT(0.4), 3000, 20, 1.0);

    FanSpeedLayerTimeSettings fan_speed_layer_time_settings; //A dummy fan speed and layer time settings.
    fan_speed_layer_time_settings.cool_min_layer_time = 0;
    fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max = 1;
    fan_speed_layer_time_settings.cool_fan_speed_min = 0;
    fan_speed_layer_time_settings.cool_fan_speed_max = 1;
    fan_speed_layer_time_settings.cool_min_speed = 0.5;
    std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder;
    fan_speed_layer_time_settings_per_extruder.push_back(fan_speed_layer_time_settings);
    //                              Slice     layer  z  layer   last        current   fan speed and layer                        combing           comb    travel  travel avoid
    //                              storage   nr        height  position    extruder  time settings                              mode              offset  avoid   distance
    gCodePlanner = new GCodePlanner(*storage, 0,     0, 0.1,    Point(0,0), 0,        fan_speed_layer_time_settings_per_extruder, CombingMode::OFF, 100,    false,  50          );
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
    GCodePathConfig configuration = storage->travel_config_per_extruder.back();
    RetractionConfig retraction_config = storage->retraction_config_per_extruder.back();
    gCodePlanner->addExtrusionMove(Point(0, 0), &configuration, SpaceFillType::Lines, 1.0f); //Need to have at least one path to have a configuration.
    GCodePath* travel_path = gCodePlanner->getLatestPathWithConfig(&configuration, SpaceFillType::None);
    gCodePlanner->addTravel_simple(Point(10, 0), travel_path);
    TimeMaterialEstimates before_retract = gCodePlanner->computeNaiveTimeEstimates();
    travel_path->retract = true;
    TimeMaterialEstimates after_retract = gCodePlanner->computeNaiveTimeEstimates();
    TimeMaterialEstimates estimate_one_retraction = after_retract - before_retract;
    double retract_unretract_time = retraction_config.distance / retraction_config.primeSpeed;
    TimeMaterialEstimates estimate_one_retraction_expected(0,retract_unretract_time * 0.5,retract_unretract_time * 0.5,0);
    verifyEstimates(estimate_one_retraction,estimate_one_retraction_expected,"One retraction");
}

void GCodePlannerTest::verifyEstimates(const TimeMaterialEstimates& observed,const TimeMaterialEstimates& expected,std::string test_description)
{
    //Check each of the four estimates in the TimeMaterialEstimate instances.
    {
        std::stringstream ss;
        ss << test_description << ": Extrude time is " << observed.getExtrudeTime() << " instead of the expected " << expected.getExtrudeTime();
        CPPUNIT_ASSERT_MESSAGE(ss.str(),observed.getExtrudeTime() - expected.getExtrudeTime() < expected.getExtrudeTime() * ALLOWED_ESTIMATE_ERROR + 0.001);
    }
    {
        std::stringstream ss;
        ss << test_description << ": Unretracted travel time is " << (observed.getTotalUnretractedTime() - observed.getExtrudeTime()) << " instead of the expected " << (expected.getTotalUnretractedTime() - expected.getExtrudeTime());
        CPPUNIT_ASSERT_MESSAGE(ss.str(),observed.getTotalUnretractedTime() - observed.getExtrudeTime() - (expected.getTotalUnretractedTime() - expected.getExtrudeTime()) < (expected.getTotalUnretractedTime() - expected.getExtrudeTime()) * ALLOWED_ESTIMATE_ERROR + 0.001);
    }
    {
        std::stringstream ss;
        ss << test_description << ": Retracted travel time is " << (observed.getTotalTime() - observed.getTotalUnretractedTime()) << " instead of the expected " << (expected.getTotalTime() - expected.getTotalUnretractedTime());
        CPPUNIT_ASSERT_MESSAGE(ss.str(),observed.getTotalTime() - observed.getTotalUnretractedTime() - (expected.getTotalTime() - expected.getTotalUnretractedTime()) < (expected.getTotalTime() - expected.getTotalUnretractedTime()) * ALLOWED_ESTIMATE_ERROR + 0.001);
    }
    {
        std::stringstream ss;
        ss << test_description << ": Material used is " << observed.getMaterial() << " instead of the expected " << expected.getMaterial();
        CPPUNIT_ASSERT_MESSAGE(ss.str(),observed.getMaterial() - expected.getMaterial() < expected.getMaterial() * ALLOWED_ESTIMATE_ERROR + 0.001);
    }
}

}