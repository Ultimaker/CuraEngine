//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "GCodeExportTest.h"

namespace cura
{
CPPUNIT_TEST_SUITE_REGISTRATION(GCodeExportTest);

void GCodeExportTest::setUp()
{
    output.clear();
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
        ";A Chilean chinchilla's chin chilly"), output.str());
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

} //namespace cura