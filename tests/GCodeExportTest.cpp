//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <gtest/gtest.h>

#include "../src/settings/types/LayerIndex.h"
#include "../src/utils/Date.h" //To check the Griffin header.
#include "../src/Application.h" //To set up a slice with settings.
#include "../src/gcodeExport.h" //The unit under test.
#include "../src/Slice.h" //To set up a slice with settings.

namespace cura
{

/*
 * Fixture that provides a GCodeExport instance in a certain base state.
 */
class GCodeExportTest : public testing::Test
{
public:
    /*
     * An export class to test with.
     */
    GCodeExport gcode;

    /*
     * A stream to capture the output of the g-code export.
     */
    std::stringstream output;

    void SetUp()
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
        Application::getInstance().current_slice = new Slice(1);
    }

    void TearDown()
    {
        delete Application::getInstance().current_slice;
    }
};

TEST_F(GCodeExportTest, CommentEmpty)
{
    gcode.writeComment("");
    EXPECT_EQ(std::string(";\n"), output.str()) << "Semicolon and newline must exist but it must be empty for the rest.";
}

TEST_F(GCodeExportTest, CommentSimple)
{
    gcode.writeComment("extrude harder");
    EXPECT_EQ(std::string(";extrude harder\n"), output.str()) << "Message must be preceded by a semicolon and ends with a newline..";
}

TEST_F(GCodeExportTest, CommentMultiLine)
{
    gcode.writeComment("If you catch a chinchilla in Chile\n"
        "And cut off its beard, willy-nilly\n"
        "You can honestly say\n"
        "You made on that day\n"
        "A Chilean chinchilla's chin chilly");
    EXPECT_EQ(std::string(";If you catch a chinchilla in Chile\n"
        ";And cut off its beard, willy-nilly\n"
        ";You can honestly say\n"
        ";You made on that day\n"
        ";A Chilean chinchilla's chin chilly\n"), output.str()) << "Each line must be preceded by a semicolon.";
}

TEST_F(GCodeExportTest, CommentMultiple)
{
    gcode.writeComment("Thunderbolt and lightning");
    gcode.writeComment("Very very frightening me");
    gcode.writeComment(" - Galileo (1638)");
    EXPECT_EQ(std::string(";Thunderbolt and lightning\n"
        ";Very very frightening me\n"
        "; - Galileo (1638)\n"), output.str()) << "Semicolon before each line, and newline in between.";
}

TEST_F(GCodeExportTest, CommentTimeZero)
{
    gcode.writeTimeComment(0);
    EXPECT_EQ(std::string(";TIME_ELAPSED:0.000000\n"), output.str());
}

TEST_F(GCodeExportTest, CommentTimeInteger)
{
    gcode.writeTimeComment(42);
    EXPECT_EQ(std::string(";TIME_ELAPSED:42.000000\n"), output.str()) << "The time must be fixed-radix to the microsecond.";
}

TEST_F(GCodeExportTest, CommentTimeFloatRoundingError)
{
    gcode.writeTimeComment(0.3);
    EXPECT_EQ(std::string(";TIME_ELAPSED:0.300000\n"), output.str()) << "Don't output up to the precision of rounding errors.";
}

TEST_F(GCodeExportTest, CommentTypeAllTypesCovered)
{
    for (PrintFeatureType type = PrintFeatureType(0); type < PrintFeatureType::NumPrintFeatureTypes; type = PrintFeatureType(static_cast<size_t>(type) + 1))
    {
        gcode.writeTypeComment(type);
        if (type == PrintFeatureType::MoveCombing || type == PrintFeatureType::MoveRetraction)
        {
            EXPECT_EQ(std::string(""), output.str()) << "Travel moves shouldn't output a type.";
        }
        else if (type == PrintFeatureType::NoneType)
        {
            EXPECT_EQ(std::string(""), output.str()) << "NoneType shouldn't output a type.";
        }
        else
        {
            EXPECT_EQ(std::string(";TYPE:"), output.str().substr(0, 6)) << "Type " << static_cast<size_t>(type) << " is not implemented.";
        }
        output.str(""); //Reset so that our next measurement is clean again.
        output << std::fixed;
    }
}

TEST_F(GCodeExportTest, CommentLayer)
{
    gcode.writeLayerComment(9);
    EXPECT_EQ(std::string(";LAYER:9\n"), output.str()) << "Put the correct prefix and a newline afterwards.";
}

TEST_F(GCodeExportTest, CommentLayerNegative)
{
    gcode.writeLayerComment(-3);
    EXPECT_EQ(std::string(";LAYER:-3\n"), output.str());
}

TEST_F(GCodeExportTest, CommentLayerCount)
{
    gcode.writeLayerCountComment(5);
    EXPECT_EQ(std::string(";LAYER_COUNT:5\n"), output.str());
}

/*
 * Parameterized test with different numbers of extruders.
 */
class GriffinHeaderTest : public testing::TestWithParam<size_t>
{
public:
    /*
     * An export class to test with.
     */
    GCodeExport gcode;

    /*
     * A stream to capture the output of the g-code export.
     */
    std::stringstream output;

    void SetUp()
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

    void TearDown()
    {
        delete Application::getInstance().current_slice;
    }
};

TEST_P(GriffinHeaderTest, HeaderGriffinFormat)
{
    const size_t num_extruders = GetParam();
    gcode.flavor = EGCodeFlavor::GRIFFIN;
    gcode.extruder_count = num_extruders;
    for (size_t extruder_index = 0; extruder_index < num_extruders; extruder_index++)
    {
        Application::getInstance().current_slice->scene.extruders.emplace_back(extruder_index, nullptr);
        ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders.back();
        train.settings.add("machine_nozzle_size", "0.4");
        train.settings.add("machine_nozzle_id", "TestNozzle");
    }

    const std::vector<bool> extruder_is_used(num_extruders, true);
    std::istringstream result(gcode.getFileHeader(extruder_is_used));
    std::string token;

    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";START_OF_HEADER"), token);
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";HEADER_VERSION:"), token.substr(0, 16)); //Actual version doesn't matter in this test.
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";FLAVOR:Griffin"), token);
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";GENERATOR.NAME:Cura_SteamEngine"), token);
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";GENERATOR.VERSION:"), token.substr(0, 19));
    EXPECT_EQ(std::string("master"), token.substr(19));
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";GENERATOR.BUILD_DATE:"), token.substr(0, 22));
    EXPECT_EQ(Date::getDate().toStringDashed(), token.substr(22));
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";TARGET_MACHINE.NAME:"), token.substr(0, 21));
    EXPECT_EQ(gcode.machine_name, token.substr(21));

    for (size_t extruder_nr = 0; extruder_nr < num_extruders; extruder_nr++)
    {
        std::getline(result, token, '\n');
        EXPECT_EQ(std::string(";EXTRUDER_TRAIN."), token.substr(0, 16));
        EXPECT_EQ(std::to_string(extruder_nr), token.substr(16, 1)); //TODO: Assumes the extruder nr is 1 digit.
        EXPECT_EQ(std::string(".INITIAL_TEMPERATURE:"), token.substr(17, 21)); //Actual temperature doesn't matter.
        std::getline(result, token, '\n');
        EXPECT_EQ(std::string(";EXTRUDER_TRAIN."), token.substr(0, 16));
        EXPECT_EQ(std::to_string(extruder_nr), token.substr(16, 1)); //TODO: Assumes the extruder nr is 1 digit.
        EXPECT_EQ(std::string(".NOZZLE.DIAMETER:0.4"), token.substr(17, 20)); //Nozzle size needs to be equal to the machine_nozzle_size setting.
        std::getline(result, token, '\n');
        EXPECT_EQ(std::string(";EXTRUDER_TRAIN."), token.substr(0, 16));
        EXPECT_EQ(std::to_string(extruder_nr), token.substr(16, 1)); //TODO: Assumes the extruder nr is 1 digit.
        EXPECT_EQ(std::string(".NOZZLE.NAME:TestNozzle"), token.substr(17, 23)); //Nozzle name needs to be equal to the machine_nozzle_id setting.
    }

    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";BUILD_PLATE.TYPE:"), token.substr(0, 18));
    EXPECT_EQ(gcode.machine_buildplate_type, token.substr(18));
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";BUILD_PLATE.INITIAL_TEMPERATURE:"), token.substr(0, 33)); //Actual temperature doesn't matter in this test.
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";PRINT.GROUPS:0"), token);
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";PRINT.SIZE.MIN.X:"), token.substr(0, 18)); //Actual bounds don't matter in this test.
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";PRINT.SIZE.MIN.Y:"), token.substr(0, 18));
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";PRINT.SIZE.MIN.Z:"), token.substr(0, 18));
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";PRINT.SIZE.MAX.X:"), token.substr(0, 18));
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";PRINT.SIZE.MAX.Y:"), token.substr(0, 18));
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";PRINT.SIZE.MAX.Z:"), token.substr(0, 18));
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";END_OF_HEADER"), token);
}

INSTANTIATE_TEST_CASE_P(GriffinHeaderTestInstantiation, GriffinHeaderTest, testing::Values(0, 1, 2, 9));

/*
 * Test the default header generation.
 */
TEST_F(GCodeExportTest, HeaderUltiGCode)
{
    gcode.flavor = EGCodeFlavor::ULTIGCODE;
    constexpr size_t num_extruders = 2;
    const std::vector<bool> extruder_is_used(num_extruders, true);
    constexpr Duration print_time = 1337;
    const std::vector<double> filament_used = {100, 200};
    for (size_t extruder_index = 0; extruder_index < num_extruders; extruder_index++)
    {
        Application::getInstance().current_slice->scene.extruders.emplace_back(extruder_index, nullptr);
        ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders.back();
        train.settings.add("machine_nozzle_size", "0.4");
    }

    std::string result = gcode.getFileHeader(extruder_is_used, &print_time, filament_used);

    EXPECT_EQ(result, ";FLAVOR:UltiGCode\n;TIME:1337\n;MATERIAL:100\n;MATERIAL2:200\n;NOZZLE_DIAMETER:0.4\n");
}

TEST_F(GCodeExportTest, HeaderRepRap)
{
    Application::getInstance().current_slice->scene.current_mesh_group->settings.add("layer_height", "0.123");
    gcode.flavor = EGCodeFlavor::REPRAP;
    gcode.extruder_attr[0].filament_area = 5.0;
    gcode.extruder_attr[1].filament_area = 4.0;
    constexpr size_t num_extruders = 2;
    const std::vector<bool> extruder_is_used(num_extruders, true);
    constexpr Duration print_time = 1337;
    const std::vector<double> filament_used = {100, 200};

    std::string result = gcode.getFileHeader(extruder_is_used, &print_time, filament_used);

    EXPECT_EQ(result, ";FLAVOR:RepRap\n;TIME:1337\n;Filament used: 0.02m, 0.05m\n;Layer height: 0.123\n");
}

TEST_F(GCodeExportTest, HeaderMarlin)
{
    Application::getInstance().current_slice->scene.current_mesh_group->settings.add("layer_height", "0.123");
    gcode.flavor = EGCodeFlavor::MARLIN;
    gcode.extruder_attr[0].filament_area = 5.0;
    gcode.extruder_attr[1].filament_area = 4.0;
    constexpr size_t num_extruders = 2;
    const std::vector<bool> extruder_is_used(num_extruders, true);
    constexpr Duration print_time = 1337;
    const std::vector<double> filament_used = {100, 200};

    std::string result = gcode.getFileHeader(extruder_is_used, &print_time, filament_used);

    EXPECT_EQ(result, ";FLAVOR:Marlin\n;TIME:1337\n;Filament used: 0.02m, 0.05m\n;Layer height: 0.123\n");
}

TEST_F(GCodeExportTest, HeaderMarlinVolumetric)
{
    Application::getInstance().current_slice->scene.current_mesh_group->settings.add("layer_height", "0.123");
    gcode.flavor = EGCodeFlavor::MARLIN_VOLUMATRIC;
    constexpr size_t num_extruders = 2;
    const std::vector<bool> extruder_is_used(num_extruders, true);
    constexpr Duration print_time = 1337;
    const std::vector<double> filament_used = {100, 200};

    std::string result = gcode.getFileHeader(extruder_is_used, &print_time, filament_used);

    EXPECT_EQ(result, ";FLAVOR:Marlin(Volumetric)\n;TIME:1337\n;Filament used: 100mm3, 200mm3\n;Layer height: 0.123\n");
}

} //namespace cura