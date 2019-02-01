//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "GCodeExportTest.h"
#include "../src/settings/types/LayerIndex.h"
#include "../src/utils/Date.h" //To check the Griffin header.
#include "../src/Application.h" //To set up a slice with settings.
#include "../src/Slice.h" //To set up a slice with settings.

namespace cura
{
CPPUNIT_TEST_SUITE_REGISTRATION(GCodeExportTest);

void GCodeExportTest::setUp()
{
    output << std::fixed;
    gcode.output_stream = &output;

    //Since GCodeExport doesn't support copying, we have to reset everything in-place.
    gcode.currentPosition = Point3(0, 0, MM2INT(20));
    gcode.layer_nr = 0;
    gcode.current_e_value = 0;
    gcode.current_extruder = 0;
    gcode.current_fan_speed = -1;
    gcode.total_print_times = std::vector<Duration>(static_cast<unsigned char>(PrintFeatureType::NumPrintFeatureTypes), 0.0);
    gcode.currentSpeed = 1;
    gcode.current_print_acceleration = -1;
    gcode.current_travel_acceleration = -1;
    gcode.current_jerk = -1;
    gcode.current_max_z_feedrate = -1;
    gcode.is_z_hopped = 0;
    gcode.setFlavor(EGCodeFlavor::MARLIN);
    gcode.initial_bed_temp = 0;
    gcode.extruder_count = 0;
    gcode.fan_number = 0;
    gcode.total_bounding_box = AABB3D();

    gcode.new_line = "\n"; //Not BFB flavour by default.
    gcode.machine_name = "Your favourite 3D printer";
    gcode.machine_buildplate_type = "Your favourite build plate";

    //Set up a scene so that we may request settings.
    Application::getInstance().current_slice = new Slice(0);
}

void GCodeExportTest::tearDown()
{
    delete Application::getInstance().current_slice;
}

void GCodeExportTest::commentEmpty()
{
    gcode.writeComment("");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("Semicolon and newline must exist but it must be empty for the rest.",
        std::string(";\n"), output.str());
}

void GCodeExportTest::commentSimple()
{
    gcode.writeComment("extrude harder");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("Message must be preceded by a semicolon and ends with a newline..",
        std::string(";extrude harder\n"), output.str());
}

void GCodeExportTest::commentMultiLine()
{
    gcode.writeComment("If you catch a chinchilla in Chile\n"
        "And cut off its beard, willy-nilly\n"
        "You can honestly say\n"
        "You made on that day\n"
        "A Chilean chinchilla's chin chilly");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("Each line must be preceded by a semicolon.",
        std::string(";If you catch a chinchilla in Chile\n"
        ";And cut off its beard, willy-nilly\n"
        ";You can honestly say\n"
        ";You made on that day\n"
        ";A Chilean chinchilla's chin chilly\n"), output.str());
}

void GCodeExportTest::commentMultiple()
{
    gcode.writeComment("Thunderbolt and lightning");
    gcode.writeComment("Very very frightening");
    gcode.writeComment(" - Galileo (1638)");
    CPPUNIT_ASSERT_EQUAL_MESSAGE("Semicolon before each line, and newline in between.",
        std::string(";Thunderbolt and lightning\n"
        ";Very very frightening\n"
        "; - Galileo (1638)\n"), output.str());
}

void GCodeExportTest::commentTimeZero()
{
    gcode.writeTimeComment(0);
    CPPUNIT_ASSERT_EQUAL(std::string(";TIME_ELAPSED:0.000000\n"), output.str());
}

void GCodeExportTest::commentTimeInteger()
{
    gcode.writeTimeComment(42);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("The time must be fixed-radix to the microsecond.",
        std::string(";TIME_ELAPSED:42.000000\n"), output.str());
}

void GCodeExportTest::commentTimeFloatRoundingError()
{
    gcode.writeTimeComment(0.3);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("Don't output up to the precision of rounding errors.",
        std::string(";TIME_ELAPSED:0.300000\n"), output.str());
}

void GCodeExportTest::commentTypeAllTypesCovered()
{
    for (PrintFeatureType type = PrintFeatureType(0); type < PrintFeatureType::NumPrintFeatureTypes; type = PrintFeatureType(static_cast<size_t>(type) + 1))
    {
        gcode.writeTypeComment(type);
        if (type == PrintFeatureType::MoveCombing || type == PrintFeatureType::MoveRetraction)
        {
            CPPUNIT_ASSERT_EQUAL_MESSAGE("Travel moves shouldn't output a type.",
                std::string(""), output.str());
        }
        else if (type == PrintFeatureType::NoneType)
        {
            CPPUNIT_ASSERT_EQUAL_MESSAGE("NoneType shouldn't output a type.",
                std::string(""), output.str());
        }
        else
        {
            std::stringstream ss;
            ss << "Type " << static_cast<size_t>(type) << " is not implemented.";
            CPPUNIT_ASSERT_EQUAL_MESSAGE(ss.str(), std::string(";TYPE:"), output.str().substr(0, 6));
        }
        output.str(""); //Reset so that our next measurement is clean again.
        output << std::fixed;
    }
}

void GCodeExportTest::commentLayer()
{
    gcode.writeLayerComment(9);
    CPPUNIT_ASSERT_EQUAL_MESSAGE("Put the correct prefix and a newline afterwards.",
        std::string(";LAYER:9\n"), output.str());
}

void GCodeExportTest::commentLayerNegative()
{
    gcode.writeLayerComment(-3);
    CPPUNIT_ASSERT_EQUAL(std::string(";LAYER:-3\n"), output.str());
}

void GCodeExportTest::commentLayerCount()
{
    gcode.writeLayerCountComment(5);
    CPPUNIT_ASSERT_EQUAL(std::string(";LAYER_COUNT:5\n"), output.str());
}

void GCodeExportTest::headerGriffinFormatNoExtruders()
{
    headerGriffinFormatCheck(0);
}

void GCodeExportTest::headerGriffinFormatCheck(const size_t num_extruders)
{
    gcode.flavor = EGCodeFlavor::GRIFFIN;
    gcode.extruder_count = num_extruders;

    const std::vector<bool> extruder_is_used(num_extruders, true);
    std::istringstream result(gcode.getFileHeader(extruder_is_used));
    std::string token;

    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";START_OF_HEADER"), token);
    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";HEADER_VERSION:"), token.substr(0, 16)); //Actual version doesn't matter in this test.
    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";FLAVOR:Griffin"), token);
    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";GENERATOR.NAME:Cura_SteamEngine"), token);
    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";GENERATOR.VERSION:"), token.substr(0, 19));
    CPPUNIT_ASSERT_EQUAL(std::string("master"), token.substr(19));
    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";GENERATOR.BUILD_DATE:"), token.substr(0, 22));
    CPPUNIT_ASSERT_EQUAL(Date::getDate().toStringDashed(), token.substr(22));
    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";TARGET_MACHINE.NAME:"), token.substr(0, 21));
    CPPUNIT_ASSERT_EQUAL(gcode.machine_name, token.substr(21));

    for (size_t extruder_nr = 0; extruder_nr < num_extruders; extruder_nr++)
    {
        std::getline(result, token, '\n');
        CPPUNIT_ASSERT_EQUAL(std::string(";EXTRUDER_TRAIN."), token.substr(0, 16));
        CPPUNIT_ASSERT_EQUAL(std::to_string(extruder_nr), token.substr(16, 1)); //TODO: Assumes the extruder nr is 1 digit.
        CPPUNIT_ASSERT_EQUAL(std::string(".INITIAL_TEMPERATURE:"), token.substr(17, 21)); //Actual temperature doesn't matter.
        std::getline(result, token, '\n');
        CPPUNIT_ASSERT_EQUAL(std::string(";EXTRUDER_TRAIN."), token.substr(0, 16));
        CPPUNIT_ASSERT_EQUAL(std::to_string(extruder_nr), token.substr(16, 1)); //TODO: Assumes the extruder nr is 1 digit.
        CPPUNIT_ASSERT_EQUAL(std::string(".NOZZLE.DIAMETER:"), token.substr(17, 17)); //Actual nozzle diameter doesn't matter.
        std::getline(result, token, '\n');
        CPPUNIT_ASSERT_EQUAL(std::string(";EXTRUDER_TRAIN."), token.substr(0, 16));
        CPPUNIT_ASSERT_EQUAL(std::to_string(extruder_nr), token.substr(16, 1)); //TODO: Assumes the extruder nr is 1 digit.
        CPPUNIT_ASSERT_EQUAL(std::string(".NOZZLE.NAME:"), token.substr(17, 13)); //Actual nozzle name doesn't matter.
    }

    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";BUILD_PLATE.TYPE:"), token.substr(0, 18));
    CPPUNIT_ASSERT_EQUAL(gcode.machine_buildplate_type, token.substr(18));
    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";BUILD_PLATE.INITIAL_TEMPERATURE:"), token.substr(0, 33)); //Actual temperature doesn't matter in this test.
    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";PRINT.GROUPS:0"), token);
    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";PRINT.SIZE.MIN.X:"), token.substr(0, 18)); //Actual bounds don't matter in this test.
    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";PRINT.SIZE.MIN.Y:"), token.substr(0, 18));
    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";PRINT.SIZE.MIN.Z:"), token.substr(0, 18));
    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";PRINT.SIZE.MAX.X:"), token.substr(0, 18));
    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";PRINT.SIZE.MAX.Y:"), token.substr(0, 18));
    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";PRINT.SIZE.MAX.Z:"), token.substr(0, 18));
    std::getline(result, token, '\n');
    CPPUNIT_ASSERT_EQUAL(std::string(";END_OF_HEADER"), token);
}

} //namespace cura