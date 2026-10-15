// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "gcode_export/gcodeExport.h" // The unit under test.

#include <cmath>

#include <gtest/gtest.h>

#include "Application.h" // To set up a slice with settings.
#include "RetractionConfig.h" // For extruder switch tests.
#include "Slice.h" // To set up a slice with settings.
#include "WipeScriptConfig.h" // For wipe script tests.
#include "arcus/MockCommunication.h" // To prevent calls to any missing Communication class.
#include "gcode_export/GCodePart.h"
#include "utils/Coord_t.h"
#include "utils/Date.h" // To check the Griffin header.

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{

/*
 * Fixture that provides a GCodeExport instance in a certain base state.
 */
// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
class GCodeExportTest : public testing::Test
{
public:
    /*
     * An export class to test with.
     */
    GCodeExport gcode;

    /*
     * Mock away the communication channel where layer data is output by this
     * class.
     */
    std::shared_ptr<MockCommunication> mock_communication;

    /*
     * Temporary stream to place the generated gcode into
     */
    std::stringstream output_;

    void selectFlavor(const EGCodeFlavor& flavor)
    {
        gcode.setFlavor(flavor);
        gcode.invalidateMotionState();
    }

    void SetUp() override
    {
        // Since GCodeExport doesn't support copying, we have to reset everything in-place.
        gcode.current_position_ = Point3LL(0, 0, MM2INT(20));
        gcode.layer_nr_ = 0;
        gcode.current_e_value_ = 0;
        gcode.current_e_offset_ = 0;
        gcode.current_extruder_ = 0;
        gcode.total_print_times_ = std::vector<Duration>(static_cast<unsigned char>(PrintFeatureType::NumPrintFeatureTypes), 0.0);
        gcode.current_speed_ = 1.0;
        gcode.current_print_acceleration_ = -1.0;
        gcode.current_travel_acceleration_ = -1.0;
        gcode.current_jerk_ = -1.0;
        gcode.is_z_hopped_ = 0;
        gcode.setFlavor(EGCodeFlavor::MARLIN);
        gcode.bed_temperature_ = 0;
        gcode.initial_bed_temp_ = 0;
        gcode.total_bounding_box_ = AABB3D();
        gcode.current_layer_z_ = 0;
        gcode.relative_extrusion_ = false;

        gcode.new_line_ = "\n"; // Not BFB flavour by default.
        gcode.machine_name_ = "Your favourite 3D printer";

        // Set up a scene so that we may request settings.
        Application::getInstance().current_slice_ = std::make_shared<Slice>(1);
        Settings& settings = Application::getInstance().current_slice_->scene.settings;
        settings.add("acceleration_enabled", "false");
        settings.add("jerk_enabled", "false");
        settings.add("acceleration_travel_enabled", "true");
        settings.add("jerk_travel_enabled", "true");
        settings.add("machine_max_feedrate_x", "1000");
        settings.add("machine_max_feedrate_y", "1000");
        settings.add("machine_max_feedrate_z", "40");
        settings.add("machine_max_feedrate_e", "1000");
        settings.add("machine_max_acceleration_x", "10000");
        settings.add("machine_max_acceleration_y", "10000");
        settings.add("machine_max_acceleration_z", "100");
        settings.add("machine_max_acceleration_e", "10000");
        settings.add("machine_max_jerk_xy", "20");
        settings.add("machine_max_jerk_z", "0.4");
        settings.add("machine_max_jerk_e", "5");
        settings.add("machine_minimum_feedrate", "0");
        settings.add("machine_acceleration", "1000");
        settings.add("speed_travel", "100");
        settings.add("speed_z_hop", "1");
        settings.add("acceleration_travel", "1000");
        settings.add("acceleration_print", "500");
        settings.add("jerk_travel", "20");
        settings.add("jerk_print", "10");
        settings.add("retraction_retract_speed", "50");
        settings.add("retraction_prime_speed", "50");
        settings.add("material_max_flowrate", "1000");
        settings.add("layer_height", "0.2");
        mock_communication = std::make_shared<MockCommunication>();
        Application::getInstance().communication_ = mock_communication;
        refreshMotionSettings();
    }

    void TearDown() override
    {
        Application::getInstance().communication_ = nullptr;
    }

    std::stringstream& output()
    {
        for (const std::shared_ptr<GCodePart>& gcode_part : gcode.gcode_parts_)
        {
            output_ << gcode_part->str();
        }
        gcode.gcode_parts_.clear();
        gcode.prepareNewFixedGCodePart();
        return output_;
    }

    Settings& motionSettings()
    {
        return Application::getInstance().current_slice_->scene.settings;
    }

    ExtruderTrain& motionExtruder()
    {
        auto& scene = Application::getInstance().current_slice_->scene;
        if (scene.extruders.empty())
        {
            scene.extruders.emplace_back(0, &scene.settings);
            scene.extruders.front().initializeMotionSettings();
        }
        return scene.extruders.front();
    }

    void prepareMotion(double filament_area = 2.0)
    {
        motionExtruder();
        refreshMotionSettings();
        gcode.current_position_ = Point3LL(0, 0, 0);
        gcode.current_layer_z_ = 0;
        gcode.extruder_attr_[0].filament_area_ = filament_area;
        gcode.extruder_attr_[0].machine_firmware_retract_ = false;
        gcode.extruder_attr_[0].last_retraction_prime_speed_ = 50.0;
        gcode.estimate_calculator_.setFirmwareDefaults(motionSettings());
        gcode.estimate_calculator_.setPosition(TimeEstimateCalculator::Position(0, 0, 0, 0));
    }

    void refreshMotionSettings()
    {
        gcode.initializeMotionSettings();
        gcode.invalidateMotionState();
    }

    void addPreSetupSettings()
    {
        Settings& settings = motionSettings();
        settings.add("machine_gcode_flavor", "RepRap (Marlin/Sprinter)");
        settings.add("machine_use_extruder_offset_to_offset_coords", "false");
        settings.add("ppr_enable", "false");
        settings.add("material_diameter", "2.85");
        settings.add("machine_extruder_cooling_fan_number", "0");
        settings.add("machine_firmware_retract", "false");
        settings.add("machine_name", "Motion cache test");
        settings.add("relative_extrusion", "false");
        settings.add("machine_always_write_active_tool", "false");
        settings.add("material_bed_temp_prepend", "false");
        settings.add("material_bed_temperature_layer_0", "0");
        settings.add("material_bed_temperature", "0");
        settings.add("machine_extruder_prestart_code", "");
        settings.add("machine_extruder_start_code", "");
        settings.add("machine_extruder_end_code", "");
        settings.add("machine_extruder_start_code_duration", "0");
        settings.add("machine_extruder_end_code_duration", "0");
        settings.add("machine_extruder_change_duration", "0");
        settings.add("retraction_enable", "false");
    }

    void stationaryPrime(double volume, bool relative = false)
    {
        gcode.relative_extrusion_ = relative;
        gcode.addExtraPrimeAmount(volume);
        gcode.writeUnretractionAndPrime();
    }

    void useFirmwareRetraction()
    {
        gcode.extruder_attr_[0].machine_firmware_retract_ = true;
    }

    std::vector<Duration> estimateMotion()
    {
        return gcode.estimate_calculator_.calculate();
    }

    void checkCompensationReset(bool relative, bool volumetric)
    {
        prepareMotion();
        gcode.relative_extrusion_ = relative;
        if (volumetric)
        {
            selectFlavor(EGCodeFlavor::MARLIN_VOLUMATRIC);
        }
        gcode.setFlowRateExtrusionSettings(10.0, 1.0);
        gcode.writeExtrusion(Point3LL(1000, 0, 0), { 2.0, 1000.0, 20.0 }, 2.0, PrintFeatureType::Infill, true);
        output();
        output_.str("");
        gcode.estimate_calculator_.reset();

        gcode.resetExtrusionValue();
        EXPECT_DOUBLE_EQ(gcode.getTotalFilamentUsed(0), 2.0);
        RetractionConfig config{};
        config.distance = 1.0;
        config.speed = 50.0;
        config.primeSpeed = 50.0;
        gcode.writeRetraction(config, true);
        const std::string reset_command = relative ? "" : "G92 E0\n";
        EXPECT_EQ(output().str(), reset_command + (volumetric ? "G1 F3000 E-2\n" : "G1 F3000 E-1\n"));

        TimeEstimateCalculator reference;
        reference.setFirmwareDefaults(motionSettings());
        reference.setPosition(TimeEstimateCalculator::Position(1.0, 0.0, 0.0, 0.0));
        reference.plan(TimeEstimateCalculator::Position(1.0, 0.0, 0.0, -1.0), 50.0, PrintFeatureType::StationaryRetractUnretract);
        const size_t feature = static_cast<size_t>(PrintFeatureType::StationaryRetractUnretract);
        EXPECT_DOUBLE_EQ(static_cast<double>(estimateMotion()[feature]), static_cast<double>(reference.calculate()[feature]));

        output_.str("");
        gcode.writeTravel(Point3LL(2000, 0, 0), { 2.0, 1000.0, 20.0 });
        EXPECT_EQ(output().str(), "G0 F120 X2 Y0.00\n");
        output_.str("");
        gcode.writeUnretractionAndPrime();
        EXPECT_EQ(output().str(), relative ? (volumetric ? "G1 F3000 E2\n" : "G1 F3000 E1\n") : "G1 F3000 E0\n");
    }
};
// NOLINTEND(misc-non-private-member-variables-in-classes)

TEST_F(GCodeExportTest, CommentEmpty)
{
    gcode.writeComment("");
    EXPECT_EQ(std::string(";\n"), output().str()) << "Semicolon and newline must exist but it must be empty for the rest.";
}

TEST_F(GCodeExportTest, CommentSimple)
{
    gcode.writeComment("extrude harder");
    EXPECT_EQ(std::string(";extrude harder\n"), output().str()) << "Message must be preceded by a semicolon and ends with a newline..";
}

TEST_F(GCodeExportTest, CommentMultiLine)
{
    gcode.writeComment("If you catch a chinchilla in Chile\n"
                       "And cut off its beard, willy-nilly\n"
                       "You can honestly say\n"
                       "You made on that day\n"
                       "A Chilean chinchilla's chin chilly");
    EXPECT_EQ(
        std::string(";If you catch a chinchilla in Chile\n"
                    ";And cut off its beard, willy-nilly\n"
                    ";You can honestly say\n"
                    ";You made on that day\n"
                    ";A Chilean chinchilla's chin chilly\n"),
        output().str())
        << "Each line must be preceded by a semicolon.";
}

TEST_F(GCodeExportTest, CommentMultiple)
{
    gcode.writeComment("Thunderbolt and lightning");
    gcode.writeComment("Very very frightening me");
    gcode.writeComment(" - Galileo (1638)");
    EXPECT_EQ(
        std::string(";Thunderbolt and lightning\n"
                    ";Very very frightening me\n"
                    "; - Galileo (1638)\n"),
        output().str())
        << "Semicolon before each line, and newline in between.";
}

TEST_F(GCodeExportTest, CommentTimeZero)
{
    gcode.writeTimeComment(0);
    EXPECT_EQ(std::string(";TIME_ELAPSED:0.000000\n"), output().str());
}

TEST_F(GCodeExportTest, CommentTimeInteger)
{
    gcode.writeTimeComment(42);
    EXPECT_EQ(std::string(";TIME_ELAPSED:42.000000\n"), output().str()) << "The time must be fixed-radix to the microsecond.";
}

TEST_F(GCodeExportTest, CommentTimeFloatRoundingError)
{
    gcode.writeTimeComment(0.3);
    EXPECT_EQ(std::string(";TIME_ELAPSED:0.300000\n"), output().str()) << "Don't output up to the precision of rounding errors.";
}

TEST_F(GCodeExportTest, CommentTypeAllTypesCovered)
{
    for (auto type = static_cast<PrintFeatureType>(0); type < PrintFeatureType::NumPrintFeatureTypes; type = static_cast<PrintFeatureType>(static_cast<size_t>(type) + 1))
    {
        gcode.writeTypeComment(type);
        if (type == PrintFeatureType::MoveUnretracted || type == PrintFeatureType::MoveRetracted || type == PrintFeatureType::MoveWhileRetracting
            || type == PrintFeatureType::MoveWhileUnretracting || type == PrintFeatureType::StationaryRetractUnretract)
        {
            EXPECT_EQ(std::string(""), output().str()) << "Travel moves shouldn't output a type.";
        }
        else if (type == PrintFeatureType::NoneType)
        {
            EXPECT_EQ(std::string(""), output().str()) << "NoneType shouldn't output a type.";
        }
        else
        {
            EXPECT_EQ(std::string(";TYPE:"), output().str().substr(0, 6)) << "Type " << static_cast<size_t>(type) << " is not implemented.";
        }
        output().str(""); // Reset so that our next measurement is clean again.
    }
}

TEST_F(GCodeExportTest, CommentLayer)
{
    gcode.writeLayerComment(9);
    EXPECT_EQ(std::string(";LAYER:9\n"), output().str()) << "Put the correct prefix and a newline afterwards.";
}

TEST_F(GCodeExportTest, CommentLayerNegative)
{
    gcode.writeLayerComment(-3);
    EXPECT_EQ(std::string(";LAYER:-3\n"), output().str());
}

TEST_F(GCodeExportTest, CommentLayerCount)
{
    gcode.writeLayerCountComment(5);
    EXPECT_EQ(std::string(";LAYER_COUNT:5\n"), output().str());
}

/*
 * Parameterized test with different numbers of extruders.
 */
// NOLINTBEGIN(misc-non-private-member-variables-in-classes)
class GriffinHeaderTest : public testing::TestWithParam<size_t>
{
public:
    /*
     * An export class to test with.
     */
    GCodeExport gcode;

    void SetUp() override
    {
        // Since GCodeExport doesn't support copying, we have to reset everything in-place.
        gcode.current_position_ = Point3LL(0, 0, MM2INT(20));
        gcode.layer_nr_ = 0;
        gcode.current_e_value_ = 0;
        gcode.current_extruder_ = 0;
        gcode.total_print_times_ = std::vector<Duration>(static_cast<unsigned char>(PrintFeatureType::NumPrintFeatureTypes), 0.0);
        gcode.current_speed_ = 1.0;
        gcode.current_print_acceleration_ = -1.0;
        gcode.current_travel_acceleration_ = -1.0;
        gcode.current_jerk_ = -1.0;
        gcode.is_z_hopped_ = 0;
        gcode.setFlavor(EGCodeFlavor::MARLIN);
        gcode.initial_bed_temp_ = 0;
        gcode.bed_temperature_ = 0;
        gcode.total_bounding_box_ = AABB3D();

        gcode.new_line_ = "\n"; // Not BFB flavour by default.
        gcode.machine_name_ = "Your favourite 3D printer";

        // Set up a scene so that we may request settings.
        Application::getInstance().current_slice_ = std::make_shared<Slice>(0);
    }
};
// NOLINTEND(misc-non-private-member-variables-in-classes)

TEST_P(GriffinHeaderTest, HeaderGriffinFormat)
{
    const size_t num_extruders = GetParam();
    gcode.flavor_ = EGCodeFlavor::GRIFFIN;
    for (size_t extruder_index = 0; extruder_index < num_extruders; extruder_index++)
    {
        Application::getInstance().current_slice_->scene.extruders.emplace_back(extruder_index, nullptr);
        ExtruderTrain& train = Application::getInstance().current_slice_->scene.extruders.back();
        train.settings_.add("machine_nozzle_size", "0.4");
        train.settings_.add("machine_nozzle_id", "TestNozzle");
    }

    const std::vector<bool> extruder_is_used(num_extruders, true);
    std::istringstream result(gcode.getFileHeader(extruder_is_used));
    std::string token;

    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";START_OF_HEADER"), token);
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";HEADER_VERSION:"), token.substr(0, 16)); // Actual version doesn't matter in this test.
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";FLAVOR:Griffin"), token);
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";GENERATOR.NAME:Cura_SteamEngine"), token);
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";GENERATOR.VERSION:"), token.substr(0, 19));
    EXPECT_EQ(std::string(CURA_ENGINE_VERSION), token.substr(19));
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";GENERATOR.BUILD_DATE:"), token.substr(0, 22));
    EXPECT_EQ(Date::getDate().toStringDashed(), token.substr(22));
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";TARGET_MACHINE.NAME:"), token.substr(0, 21));
    EXPECT_EQ(gcode.machine_name_, token.substr(21));

    for (size_t extruder_nr = 0; extruder_nr < num_extruders; extruder_nr++)
    {
        std::getline(result, token, '\n');
        EXPECT_EQ(std::string(";EXTRUDER_TRAIN."), token.substr(0, 16));
        EXPECT_EQ(std::to_string(extruder_nr), token.substr(16, 1)); // TODO: Assumes the extruder nr is 1 digit.
        EXPECT_EQ(std::string(".INITIAL_TEMPERATURE:"), token.substr(17, 21)); // Actual temperature doesn't matter.
        std::getline(result, token, '\n');
        EXPECT_EQ(std::string(";EXTRUDER_TRAIN."), token.substr(0, 16));
        EXPECT_EQ(std::to_string(extruder_nr), token.substr(16, 1)); // TODO: Assumes the extruder nr is 1 digit.
        EXPECT_EQ(std::string(".NOZZLE.DIAMETER:0.4"), token.substr(17, 20)); // Nozzle size needs to be equal to the machine_nozzle_size setting.
        std::getline(result, token, '\n');
        EXPECT_EQ(std::string(";EXTRUDER_TRAIN."), token.substr(0, 16));
        EXPECT_EQ(std::to_string(extruder_nr), token.substr(16, 1)); // TODO: Assumes the extruder nr is 1 digit.
        EXPECT_EQ(std::string(".NOZZLE.NAME:TestNozzle"), token.substr(17, 23)); // Nozzle name needs to be equal to the machine_nozzle_id setting.
    }

    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";BUILD_PLATE.INITIAL_TEMPERATURE:"), token.substr(0, 33)); // Actual temperature doesn't matter in this test.
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";PRINT.TIME:0"), token);
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";PRINT.GROUPS:0"), token);
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";PRINT.SIZE.MIN.X:"), token.substr(0, 18)); // Actual bounds don't matter in this test.
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
    EXPECT_EQ(std::string(";SLICE_UUID:"), token.substr(0, 18));
    std::getline(result, token, '\n');
    EXPECT_EQ(std::string(";END_OF_HEADER"), token);
}

INSTANTIATE_TEST_SUITE_P(GriffinHeaderTestInstantiation, GriffinHeaderTest, testing::Values(0, 1, 2, 9));

/*
 * Test the default header generation.
 */
TEST_F(GCodeExportTest, HeaderUltiGCode)
{
    gcode.flavor_ = EGCodeFlavor::ULTIGCODE;
    constexpr size_t num_extruders = 2;
    const std::vector<bool> extruder_is_used(num_extruders, true);
    const std::vector<double> filament_used = { 100, 200 };
    for (size_t extruder_index = 0; extruder_index < num_extruders; extruder_index++)
    {
        Application::getInstance().current_slice_->scene.extruders.emplace_back(extruder_index, nullptr);
        ExtruderTrain& train = Application::getInstance().current_slice_->scene.extruders.back();
        train.settings_.add("machine_nozzle_size", "0.4");
    }
    gcode.total_bounding_box_ = AABB3D(Point3LL(0, 0, 0), Point3LL(1000, 1000, 1000));

    std::string result = gcode.getFileHeader(extruder_is_used, filament_used);

    EXPECT_EQ(
        result,
        ";FLAVOR:UltiGCode\n;TIME:0\n;MATERIAL:100\n;MATERIAL2:200\n;NOZZLE_DIAMETER:0.4\n;MINX:0\n;MINY:0\n;MINZ:0\n;MAXX:1\n;"
        "MAXY:1\n;MAXZ:1\n;TARGET_MACHINE.NAME:Your favourite 3D printer\n");
}

TEST_F(GCodeExportTest, HeaderRepRap)
{
    Application::getInstance().current_slice_->scene.current_mesh_group->settings.add("layer_height", "0.123");
    gcode.flavor_ = EGCodeFlavor::REPRAP;
    gcode.extruder_attr_[0].filament_area_ = 5.0;
    gcode.extruder_attr_[1].filament_area_ = 4.0;
    constexpr size_t num_extruders = 2;
    const std::vector<bool> extruder_is_used(num_extruders, true);
    const std::vector<double> filament_used = { 100, 200 };
    gcode.total_bounding_box_ = AABB3D(Point3LL(0, 0, 0), Point3LL(1000, 1000, 1000));

    std::string result = gcode.getFileHeader(extruder_is_used, filament_used);

    EXPECT_EQ(
        result,
        ";FLAVOR:RepRap\n;TIME:0\n;Filament used: 0.02m, 0.05m\n;Layer height: "
        "0.123\n;MINX:0\n;MINY:0\n;MINZ:0\n;MAXX:1\n;MAXY:1\n;MAXZ:1\n;TARGET_MACHINE.NAME:Your favourite 3D printer\n");
}

TEST_F(GCodeExportTest, HeaderMarlin)
{
    Application::getInstance().current_slice_->scene.current_mesh_group->settings.add("layer_height", "0.123");
    gcode.flavor_ = EGCodeFlavor::MARLIN;
    gcode.extruder_attr_[0].filament_area_ = 5.0;
    gcode.extruder_attr_[1].filament_area_ = 4.0;
    constexpr size_t num_extruders = 2;
    const std::vector<bool> extruder_is_used(num_extruders, true);
    const std::vector<double> filament_used = { 100, 200 };
    gcode.total_bounding_box_ = AABB3D(Point3LL(0, 0, 0), Point3LL(1000, 1000, 1000));

    std::string result = gcode.getFileHeader(extruder_is_used, filament_used);

    EXPECT_EQ(
        result,
        ";FLAVOR:Marlin\n;TIME:0\n;Filament used: 0.02m, 0.05m\n;Layer height: "
        "0.123\n;MINX:0\n;MINY:0\n;MINZ:0\n;MAXX:1\n;MAXY:1\n;MAXZ:1\n;TARGET_MACHINE.NAME:Your favourite 3D printer\n");
}

TEST_F(GCodeExportTest, HeaderMarlinVolumetric)
{
    Application::getInstance().current_slice_->scene.current_mesh_group->settings.add("layer_height", "0.123");
    gcode.flavor_ = EGCodeFlavor::MARLIN_VOLUMATRIC;
    constexpr size_t num_extruders = 2;
    const std::vector<bool> extruder_is_used(num_extruders, true);
    const std::vector<double> filament_used = { 100, 200 };
    gcode.total_bounding_box_ = AABB3D(Point3LL(0, 0, 0), Point3LL(1000, 1000, 1000));

    std::string result = gcode.getFileHeader(extruder_is_used, filament_used);

    EXPECT_EQ(
        result,
        ";FLAVOR:Marlin(Volumetric)\n;TIME:0\n;Filament used: 100mm3, 200mm3\n;Layer height: "
        "0.123\n;MINX:0\n;MINY:0\n;MINZ:0\n;MAXX:1\n;MAXY:1\n;MAXZ:1\n;TARGET_MACHINE.NAME:Your favourite 3D printer\n");
}

/*
 * Test conversion from E values to millimetres and back in the case of a
 * volumetric printer.
 */
TEST_F(GCodeExportTest, EVsMmVolumetric)
{
    constexpr double filament_area = 10.0;
    gcode.extruder_attr_[0].filament_area_ = filament_area;
    gcode.is_volumetric_ = true;

    constexpr double mm3_input = 15.0;
    EXPECT_EQ(gcode.mm3ToE(mm3_input), mm3_input) << "Since the E is volumetric and the input mm3 is also volumetric, the output needs to be the same.";

    EXPECT_EQ(gcode.eToMm(200.0), 200.0 / filament_area) << "Since the E is volumetric but mm is linear, divide by the cross-sectional "
                                                            "area of the filament to convert the volume to a length.";

    constexpr double mm_input = 33.0;
    EXPECT_EQ(gcode.mmToE(mm_input), mm_input * filament_area)
        << "Since the input mm is linear but the E output must be volumetric, we need to multiply by the cross-sectional area to convert "
           "length to volume.";

    constexpr double e_input = 100.0;
    EXPECT_EQ(gcode.eToMm3(e_input, 0), e_input) << "Since the E is volumetric and mm3 is also volumetric, the output needs to be the same.";
}

/*
 * Test conversion from E values to millimetres and back in the case where the E
 * value represents the linear position of the filament.
 */
TEST_F(GCodeExportTest, EVsMmLinear)
{
    constexpr double filament_area = 10.0;
    gcode.extruder_attr_[0].filament_area_ = filament_area;
    gcode.is_volumetric_ = false;

    EXPECT_EQ(gcode.mmToE(15.0), 15.0) << "Since the E is linear and the input mm is also linear, the output needs to be the same.";
    EXPECT_EQ(gcode.eToMm(15.0), 15.0) << "Since the E is linear and the output mm is also linear, the output needs to be the same.";

    for (int x = -1000; x < 1000; x += 16)
    {
        EXPECT_DOUBLE_EQ(gcode.mmToE(gcode.eToMm(static_cast<double>(x))), static_cast<double>(x)) << "Converting back and forth should lead to the same number.";
    }

    constexpr double mm3_input = 33.0;
    EXPECT_EQ(gcode.mm3ToE(mm3_input), mm3_input / filament_area)
        << "Since the input mm3 is volumetric but the E output must be linear, we need to divide by the cross-sectional area to convert "
           "volume to length.";

    constexpr double e_input = 100.0;
    EXPECT_EQ(gcode.eToMm3(e_input, 0), e_input * filament_area) << "Since the input E is linear but the output must be volumetric, we "
                                                                    "need to multiply by cross-sectional area to convert length to volume.";
}

/*
 * Switch extruders, with the following special cases:
 * - No retraction distance.
 */
TEST_F(GCodeExportTest, SwitchExtruderSimple)
{
    Scene& scene = Application::getInstance().current_slice_->scene;

    scene.extruders.emplace_back(0, nullptr);
    ExtruderTrain& train1 = scene.extruders.back();

    train1.settings_.add("machine_extruder_prestart_code", ";PRESTART FIRST EXTRUDER");
    train1.settings_.add("machine_extruder_change_duration", "10.0");
    train1.settings_.add("machine_extruder_start_code", ";FIRST EXTRUDER START G-CODE!");
    train1.settings_.add("machine_extruder_end_code", ";FIRST EXTRUDER END G-CODE!");
    train1.settings_.add("machine_extruder_start_code_duration", "0.0");
    train1.settings_.add("machine_extruder_end_code_duration", "0.0");
    train1.settings_.add("machine_firmware_retract", "True");
    train1.settings_.add("retraction_enable", "True");

    scene.extruders.emplace_back(1, nullptr);
    ExtruderTrain& train2 = scene.extruders.back();

    train2.settings_.add("machine_extruder_prestart_code", ";PRESTART SECOND EXTRUDER");
    train2.settings_.add("machine_extruder_change_duration", "11.1");
    train2.settings_.add("machine_extruder_start_code", ";SECOND EXTRUDER START G-CODE!");
    train2.settings_.add("machine_extruder_end_code", ";SECOND EXTRUDER END G-CODE!");
    train2.settings_.add("machine_extruder_start_code_duration", "0.0");
    train2.settings_.add("machine_extruder_end_code_duration", "0.0");
    train2.settings_.add("machine_firmware_retract", "True");
    train2.settings_.add("retraction_enable", "True");

    RetractionConfig no_retraction;
    no_retraction.distance = 0;

    EXPECT_CALL(*mock_communication, setExtruderForSend(testing::_));
    EXPECT_CALL(*mock_communication, sendCurrentPosition(testing::_));
    gcode.switchExtruder(1, no_retraction);

    EXPECT_EQ(std::string("G92 E0\n;FIRST EXTRUDER END G-CODE!\n;PRESTART SECOND EXTRUDER\nT1\nG92 E0\n;SECOND EXTRUDER START G-CODE!\n"), output().str());
}

TEST_F(GCodeExportTest, WriteZHopStartZero)
{
    gcode.writeZhopStart(0);
    EXPECT_EQ(std::string(""), output().str()) << "Zero length z hop shouldn't affect gcode output.";
}

TEST_F(GCodeExportTest, WriteZHopStartDefaultSpeed)
{
    Application::getInstance().current_slice_->scene.extruders.emplace_back(0, &motionSettings());
    Application::getInstance().current_slice_->scene.extruders[gcode.current_extruder_].settings_.add("speed_z_hop", "1"); // 60mm/min.
    Application::getInstance().current_slice_->scene.current_mesh_group->settings.add("layer_height", "0.2");
    refreshMotionSettings();
    gcode.current_layer_z_ = 2000;
    constexpr coord_t hop_height = 3000;
    gcode.writeZhopStart(hop_height);
    EXPECT_EQ(std::string("G1 F60 Z5\n"), output().str());
}

TEST_F(GCodeExportTest, WriteZHopStartCustomSpeed)
{
    Application::getInstance().current_slice_->scene.extruders.emplace_back(0, &motionSettings());
    Application::getInstance().current_slice_->scene.extruders[gcode.current_extruder_].settings_.add("speed_z_hop", "1"); // 60mm/min.
    Application::getInstance().current_slice_->scene.current_mesh_group->settings.add("layer_height", "0.2");
    refreshMotionSettings();
    gcode.current_layer_z_ = 2000;
    constexpr coord_t hop_height = 3000;
    constexpr Velocity speed{ 4.0 }; // 240 mm/min.
    gcode.writeZhopStart(hop_height, SpeedDerivatives{ speed, 1000.0, 20.0 });
    EXPECT_EQ(std::string("G1 F240 Z5\n"), output().str()) << "Custom provided speed should be used.";
}

TEST_F(GCodeExportTest, WriteZHopEndZero)
{
    gcode.is_z_hopped_ = 0;
    gcode.writeZhopEnd();
    EXPECT_EQ(std::string(""), output().str()) << "Zero length z hop shouldn't affect gcode output.";
}

TEST_F(GCodeExportTest, WriteZHopEndDefaultSpeed)
{
    Application::getInstance().current_slice_->scene.extruders.emplace_back(0, &motionSettings());
    Application::getInstance().current_slice_->scene.extruders[gcode.current_extruder_].settings_.add("speed_z_hop", "1"); // 60mm/min.
    Application::getInstance().current_slice_->scene.current_mesh_group->settings.add("layer_height", "0.2");
    refreshMotionSettings();
    gcode.current_layer_z_ = 2000;
    gcode.is_z_hopped_ = 3000;
    gcode.writeZhopEnd();
    EXPECT_EQ(std::string("G1 F60 Z2\n"), output().str());
}

TEST_F(GCodeExportTest, WriteZHopEndCustomSpeed)
{
    Application::getInstance().current_slice_->scene.extruders.emplace_back(0, &motionSettings());
    Application::getInstance().current_slice_->scene.extruders[gcode.current_extruder_].settings_.add("speed_z_hop", "1");
    Application::getInstance().current_slice_->scene.current_mesh_group->settings.add("layer_height", "0.2");
    refreshMotionSettings();
    gcode.current_layer_z_ = 2000;
    gcode.is_z_hopped_ = 3000;
    constexpr Velocity speed{ 4.0 }; // 240 mm/min.
    gcode.writeZhopEnd(SpeedDerivatives{ speed, 1000.0, 20.0 });
    EXPECT_EQ(std::string("G1 F240 Z2\n"), output().str()) << "Custom provided speed should be used.";
}

TEST_F(GCodeExportTest, insertWipeScriptSingleMove)
{
    motionExtruder();
    gcode.current_position_ = Point3LL(1000, 1000, 1000);
    gcode.current_layer_z_ = 1000;
    gcode.use_extruder_offset_to_offset_coords_ = false;
    Application::getInstance().current_slice_->scene.current_mesh_group->settings.add("layer_height", "0.2");

    WipeScriptConfig config;
    config.retraction_enable = false;
    config.hop_enable = false;
    config.brush_pos_x = 2000;
    config.repeat_count = 1;
    config.move_distance = 500;
    config.move_speed = 10.0;
    config.pause = 0;

    EXPECT_CALL(*mock_communication, sendLineTo(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_)).Times(3);
    gcode.insertWipeScript(config);

    std::string token;
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string(";WIPE_SCRIPT_BEGIN"), token) << "Wipe script should always start with tag.";
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string("G0 F600 X2 Y1"), token) << "Wipe script should go to its position.";
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string("G0 X2.5 Y1"), token) << "There should be one wipe move.";
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string("G0 X1 Y1"), token) << "Wipe script should return back to position before wipe.";
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string(";WIPE_SCRIPT_END"), token) << "Wipe script should always end with tag.";
}

TEST_F(GCodeExportTest, insertWipeScriptMultipleMoves)
{
    motionExtruder();
    gcode.current_position_ = Point3LL(1000, 1000, 1000);
    gcode.current_layer_z_ = 1000;
    gcode.use_extruder_offset_to_offset_coords_ = false;
    Application::getInstance().current_slice_->scene.current_mesh_group->settings.add("layer_height", "0.2");

    WipeScriptConfig config;
    config.retraction_enable = false;
    config.hop_enable = false;
    config.brush_pos_x = 2000;
    config.repeat_count = 4;
    config.move_distance = 500;
    config.move_speed = 10.0;
    config.pause = 0;

    EXPECT_CALL(*mock_communication, sendLineTo(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_)).Times(6);
    gcode.insertWipeScript(config);

    std::string token;
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string(";WIPE_SCRIPT_BEGIN"), token) << "Wipe script should always start with tag.";
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string("G0 F600 X2 Y1"), token) << "Wipe script should go to its position.";
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string("G0 X2.5 Y1"), token);
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string("G0 X2 Y1"), token);
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string("G0 X2.5 Y1"), token);
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string("G0 X2 Y1"), token);
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string("G0 X1 Y1"), token) << "Wipe script should return back to position before wipe.";
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string(";WIPE_SCRIPT_END"), token) << "Wipe script should always end with tag.";
}

TEST_F(GCodeExportTest, insertWipeScriptOptionalDelay)
{
    motionExtruder();
    gcode.current_position_ = Point3LL(1000, 1000, 1000);
    gcode.current_layer_z_ = 1000;
    gcode.use_extruder_offset_to_offset_coords_ = false;
    Application::getInstance().current_slice_->scene.current_mesh_group->settings.add("layer_height", "0.2");

    WipeScriptConfig config;
    config.retraction_enable = false;
    config.hop_enable = false;
    config.brush_pos_x = 2000;
    config.repeat_count = 1;
    config.move_distance = 500;
    config.move_speed = 10.0;
    config.pause = 1.5; // 1.5 sec = 1500 ms.

    EXPECT_CALL(*mock_communication, sendLineTo(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_)).Times(3);
    gcode.insertWipeScript(config);

    std::string token;
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string(";WIPE_SCRIPT_BEGIN"), token) << "Wipe script should always start with tag.";
    std::getline(output(), token, '\n'); // go to wipe position
    std::getline(output(), token, '\n'); // make wipe move
    std::getline(output(), token, '\n'); // return back
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string("G4 P1500"), token) << "Wipe script should make a delay.";
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string(";WIPE_SCRIPT_END"), token) << "Wipe script should always end with tag.";
}

TEST_F(GCodeExportTest, insertWipeScriptRetractionEnable)
{
    motionExtruder();
    gcode.current_position_ = Point3LL(1000, 1000, 1000);
    gcode.current_layer_z_ = 1000;
    gcode.current_e_value_ = 100;
    gcode.use_extruder_offset_to_offset_coords_ = false;
    gcode.is_volumetric_ = false;
    gcode.current_extruder_ = 0;
    gcode.extruder_attr_[0].filament_area_ = 10.0;
    gcode.extruder_attr_[0].machine_firmware_retract_ = false;
    gcode.relative_extrusion_ = false;
    gcode.current_speed_ = 1.0;
    Application::getInstance().current_slice_->scene.current_mesh_group->settings.add("layer_height", "0.2");

    WipeScriptConfig config;
    config.retraction_enable = true;
    config.retraction_config.distance = 1;
    config.retraction_config.speed = 2.0; // 120 mm/min.
    config.retraction_config.primeSpeed = 3.0; // 180 mm/min.
    config.retraction_config.prime_volume = gcode.extruder_attr_[0].filament_area_ * 4; // 4mm in linear dimensions
    config.retraction_config.retraction_count_max = 100; // Practically no limit.
    config.retraction_config.retraction_extrusion_window = 1;
    config.retraction_config.retraction_min_travel_distance = 0; // Don't limit retractions for being too short.
    config.hop_enable = false;
    config.brush_pos_x = 2000;
    config.repeat_count = 1;
    config.move_distance = 500;
    config.move_speed = 10.0;
    config.pause = 0;

    EXPECT_CALL(*mock_communication, sendLineTo(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_)).Times(3);
    gcode.insertWipeScript(config);

    std::string token;
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string(";WIPE_SCRIPT_BEGIN"), token) << "Wipe script should always start with tag.";
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string("G1 F120 E99"), token) << "Wipe script should perform retraction with provided speed and retraction distance.";
    std::getline(output(), token, '\n'); // go to wipe position
    std::getline(output(), token, '\n'); // make wipe move
    std::getline(output(), token, '\n'); // return back
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string("G1 F180 E104"), token) << "Wipe script should make unretraction with provided speed and extra prime volume.";
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string(";WIPE_SCRIPT_END"), token) << "Wipe script should always end with tag.";
}

TEST_F(GCodeExportTest, insertWipeScriptHopEnable)
{
    motionExtruder();
    gcode.current_position_ = Point3LL(1000, 1000, 1000);
    gcode.current_layer_z_ = 1000;
    gcode.use_extruder_offset_to_offset_coords_ = false;
    gcode.current_speed_ = 1.0;
    Application::getInstance().current_slice_->scene.current_mesh_group->settings.add("layer_height", "0.2");

    WipeScriptConfig config;
    config.retraction_enable = false;
    config.hop_enable = true;
    config.hop_speed = 2.0; // 120 mm/min.
    config.hop_amount = 300;
    config.brush_pos_x = 2000;
    config.repeat_count = 1;
    config.move_distance = 500;
    config.move_speed = 10.0;
    config.pause = 0;

    EXPECT_CALL(*mock_communication, sendLineTo(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_)).Times(5);
    gcode.insertWipeScript(config);

    std::string token;
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string(";WIPE_SCRIPT_BEGIN"), token) << "Wipe script should always start with tag.";
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string("G1 F120 Z1.3"), token) << "Wipe script should perform z-hop.";
    std::getline(output(), token, '\n'); // go to wipe position
    std::getline(output(), token, '\n'); // make wipe move
    std::getline(output(), token, '\n'); // return back
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string("G1 F120 Z1"), token) << "Wipe script should return z position.";
    std::getline(output(), token, '\n');
    EXPECT_EQ(std::string(";WIPE_SCRIPT_END"), token) << "Wipe script should always end with tag.";
}
TEST_F(GCodeExportTest, SpatialControlsAreExporterGated)
{
    prepareMotion();
    motionSettings().add("machine_max_feedrate_x", "90");
    motionSettings().add("machine_max_feedrate_y", "30");
    motionSettings().add("machine_max_feedrate_z", "5");
    motionSettings().add("material_max_flowrate", "2");
    refreshMotionSettings();
    gcode.writeTravel(Point3LL(1000, 0, 0), { 100.0, 9000.0, 50.0 });
    gcode.writeExtrusion(Point3LL(2000, 0, 0), { 100.0, 9000.0, 50.0 }, 1.0, PrintFeatureType::Infill);
    gcode.writeZhopStart(1000, SpeedDerivatives{ 100.0, 9000.0, 50.0 });
    stationaryPrime(4.0);
    const std::string result = output().str();
    EXPECT_NE(result.find("G0 F1800 X1 Y0.00"), std::string::npos);
    EXPECT_NE(result.find("G1 F120 X2 Y0.00 E0.5"), std::string::npos);
    EXPECT_NE(result.find("G1 F300 Z1"), std::string::npos);
    EXPECT_EQ(result.find("M201"), std::string::npos);
    EXPECT_EQ(result.find("M202"), std::string::npos);
    EXPECT_EQ(result.find("M204"), std::string::npos);
    EXPECT_EQ(result.find("M205"), std::string::npos);
}

TEST_F(GCodeExportTest, AccelerationAndJerkControlsAreIndependent)
{
    prepareMotion();
    motionSettings().add("acceleration_enabled", "true");
    refreshMotionSettings();
    gcode.writeTravel(Point3LL(1000, 0, 0), { 30.0, 700.0, 12.0 });
    const std::string acceleration_only = output().str();
    EXPECT_NE(acceleration_only.find("M204 S700"), std::string::npos);
    EXPECT_EQ(acceleration_only.find("M205"), std::string::npos);
    output_.str("");
    motionSettings().add("acceleration_enabled", "false");
    motionSettings().add("jerk_enabled", "true");
    refreshMotionSettings();
    gcode.writeTravel(Point3LL(2000, 0, 0), { 30.0, 800.0, 11.0 });
    const std::string jerk_only = output().str();
    EXPECT_EQ(jerk_only.find("M201"), std::string::npos);
    EXPECT_EQ(jerk_only.find("M204"), std::string::npos);
    EXPECT_NE(jerk_only.find("M205 X11 Y11"), std::string::npos);
}

TEST_F(GCodeExportTest, AsymmetricXYAndExactZLimits)
{
    prepareMotion();
    motionSettings().add("acceleration_enabled", "true");
    motionSettings().add("jerk_enabled", "true");
    motionSettings().add("machine_max_feedrate_x", "90");
    motionSettings().add("machine_max_feedrate_y", "30");
    motionSettings().add("machine_max_feedrate_z", "5");
    motionSettings().add("machine_max_acceleration_x", "700");
    motionSettings().add("machine_max_acceleration_y", "300");
    motionSettings().add("machine_max_acceleration_z", "20");
    motionSettings().add("machine_max_jerk_xy", "12");
    refreshMotionSettings();
    gcode.writeTravel(Point3LL(1000, 0, 0), { 100.0, 9000.0, 50.0 });
    gcode.writeTravel(Point3LL(1000, 0, 1000), { 100.0, 9000.0, 50.0 });
    // One micron of X motion is not an exact Z-only movement.
    gcode.writeTravel(Point3LL(1001, 0, 2000), { 100.0, 9000.0, 50.0 });
    const std::string result = output().str();
    EXPECT_NE(result.find("G0 F1800 X1 Y0.00"), std::string::npos);
    EXPECT_NE(result.find("G0 F300 Z1"), std::string::npos);
    EXPECT_NE(result.find("G0 F1800 X1.001 Y0.00 Z2"), std::string::npos);
    EXPECT_NE(result.find("M204 S300"), std::string::npos);
    EXPECT_NE(result.find("M204 S20"), std::string::npos);
    EXPECT_NE(result.find("M205 X12 Y12"), std::string::npos);
}

TEST_F(GCodeExportTest, ZeroMachineLimitIsNotUnlimited)
{
    prepareMotion();
    motionSettings().add("machine_max_feedrate_y", "0");
    refreshMotionSettings();
    gcode.writeTravel(Point3LL(1000, 0, 0), { 100.0, 1000.0, 20.0 });
    EXPECT_NE(output().str().find("G0 F0 X1 Y0.00"), std::string::npos);
    EXPECT_TRUE(std::isinf(static_cast<double>(estimateMotion()[static_cast<size_t>(PrintFeatureType::MoveUnretracted)])));
}

TEST_F(GCodeExportTest, PrintingUsesMaterialAndFilamentLimits)
{
    prepareMotion();
    motionExtruder().settings_.add("material_max_flowrate", "8");
    // A MeshGroup override must not override the extruder's material setting.
    Application::getInstance().current_slice_->scene.current_mesh_group->settings.add("material_max_flowrate", "1000");
    motionSettings().add("machine_max_feedrate_e", "3");
    refreshMotionSettings();
    gcode.writeExtrusion(Point3LL(10000, 0, 0), { 100.0, 1000.0, 20.0 }, 2.0, PrintFeatureType::Infill);
    EXPECT_NE(output().str().find("G1 F180 X10 Y0.00 E10"), std::string::npos);
    motionSettings().add("machine_max_feedrate_e", "100");
    refreshMotionSettings();
    gcode.writeExtrusion(Point3LL(20000, 0, 0), { 100.0, 1000.0, 20.0 }, 2.0, PrintFeatureType::Infill);
    EXPECT_NE(output().str().find("G1 F240 X20 Y0.00 E20"), std::string::npos);
}

TEST_F(GCodeExportTest, SpatialFilamentDerivativesAreScaled)
{
    prepareMotion();
    motionSettings().add("acceleration_enabled", "true");
    motionSettings().add("jerk_enabled", "true");
    motionSettings().add("machine_max_feedrate_e", "4");
    motionSettings().add("machine_max_acceleration_e", "100");
    motionSettings().add("machine_max_jerk_e", "2");
    refreshMotionSettings();
    gcode.writeExtrusion(Point3LL(1000, 0, 0), { 100.0, 1000.0, 20.0 }, 4.0, PrintFeatureType::Infill);
    const std::string result = output().str();
    EXPECT_NE(result.find("M204 S50"), std::string::npos);
    EXPECT_NE(result.find("M201 Z100 E100"), std::string::npos);
    EXPECT_NE(result.find("M205 X1 Y1"), std::string::npos);
    EXPECT_NE(result.find("G1 F120 X1 Y0.00 E2"), std::string::npos);
}

TEST_F(GCodeExportTest, AxisLimitsAndHeadAccelerationAreRestored)
{
    prepareMotion();
    motionSettings().add("acceleration_enabled", "true");
    motionSettings().add("jerk_enabled", "true");
    refreshMotionSettings();
    gcode.writeTravel(Point3LL(0, 0, 1000), { 10.0, 5.0, 0.1 });
    stationaryPrime(2.0);
    gcode.writeTravel(Point3LL(1000, 0, 1000), { 30.0, 800.0, 12.0 });
    const std::string result = output().str();
    EXPECT_NE(result.find("M201 Z5 E10000"), std::string::npos);
    EXPECT_NE(result.find("M205 Z0.1 E5"), std::string::npos);
    EXPECT_NE(result.find("M201 Z100 E10000"), std::string::npos);
    EXPECT_NE(result.find("M205 Z0.4 E5"), std::string::npos);
    EXPECT_NE(result.find("M204 R10000"), std::string::npos);
    EXPECT_NE(result.find("M204 S800"), std::string::npos);
}

TEST_F(GCodeExportTest, TravelRetractAndPrimeIgnoreMaterialCap)
{
    prepareMotion();
    motionSettings().add("material_max_flowrate", "0");
    motionSettings().add("machine_max_feedrate_e", "5");
    refreshMotionSettings();
    gcode.writeTravel(Point3LL(1000, 0, 0), { 100.0, 1000.0, 20.0 }, 10.0);
    gcode.writeTravel(Point3LL(2000, 0, 0), { 100.0, 1000.0, 20.0 }, 0.0);
    const std::string result = output().str();
    EXPECT_NE(result.find("G0 F30 X1 Y0.00 E-10"), std::string::npos);
    EXPECT_NE(result.find("G0 X2 Y0.00 E0"), std::string::npos);
}

TEST_F(GCodeExportTest, StationaryRetractAndPrimeIgnoreMaterialCap)
{
    prepareMotion();
    motionSettings().add("material_max_flowrate", "0");
    motionSettings().add("machine_max_feedrate_e", "5");
    refreshMotionSettings();
    RetractionConfig config{};
    config.distance = 2.0;
    config.speed = 100.0;
    config.primeSpeed = 100.0;
    gcode.writeRetraction(config, true);
    stationaryPrime(4.0);
    const std::string result = output().str();
    EXPECT_NE(result.find("G1 F300 E-2"), std::string::npos);
    EXPECT_NE(result.find("G1 F300 E2"), std::string::npos);
}

TEST_F(GCodeExportTest, StationaryTravelRetractionUsesFilamentSpeed)
{
    prepareMotion();
    motionSettings().add("machine_max_feedrate_e", "5");
    refreshMotionSettings();
    gcode.writeTravel(Point3LL(0, 0, 0), { 100.0, 1000.0, 20.0 }, 2.0);
    EXPECT_EQ(output().str(), "G1 F300 E-2\n");
}

TEST_F(GCodeExportTest, VolumetricCoordinatesDoNotScaleStationaryFeedrate)
{
    prepareMotion(4.0);
    selectFlavor(EGCodeFlavor::MARLIN_VOLUMATRIC);
    motionSettings().add("machine_max_feedrate_e", "5");
    motionSettings().add("material_max_flowrate", "0");
    refreshMotionSettings();
    stationaryPrime(8.0);
    EXPECT_EQ(output().str(), "G1 F300 E8\n");
    output_.str("");
    stationaryPrime(8.0, true);
    EXPECT_EQ(output().str(), "G1 F300 E8\n");
}

TEST_F(GCodeExportTest, VolumetricPrintingConvertsFilamentLimit)
{
    prepareMotion(4.0);
    selectFlavor(EGCodeFlavor::MARLIN_VOLUMATRIC);
    motionSettings().add("machine_max_feedrate_e", "5");
    refreshMotionSettings();
    gcode.writeExtrusion(Point3LL(10000, 0, 0), { 100.0, 1000.0, 20.0 }, 8.0, PrintFeatureType::Infill);
    EXPECT_NE(output().str().find("G1 F150 X10 Y0.00 E80"), std::string::npos);
}

TEST_F(GCodeExportTest, CompensationUsesCappedFilamentFlow)
{
    prepareMotion(4.0);
    motionSettings().add("machine_max_feedrate_e", "5");
    refreshMotionSettings();
    gcode.setFlowRateExtrusionSettings(10.0, 1.0);
    gcode.writeExtrusion(Point3LL(1000, 0, 0), { 100.0, 1000.0, 20.0 }, 4.0, PrintFeatureType::Infill, true);
    const std::string result = output().str();
    // v * (1 + v) <= 5, including the first compensation offset.
    EXPECT_LE(gcode.getCurrentSpeed() * (1.0 + gcode.getCurrentSpeed()), 5.0);
    EXPECT_NE(result.find(";FLOW_RATE_COMPENSATED_OFFSET"), std::string::npos);
    EXPECT_GT(gcode.getCurrentSpeed(), 1.7);
}

TEST_F(GCodeExportTest, CompensationCancellationPreservesLogicalExtrusion)
{
    prepareMotion();
    gcode.setFlowRateExtrusionSettings(10.0, 1.0);
    gcode.writeExtrusion(Point3LL(1000, 0, 0), { 2.0, 1000.0, 20.0 }, 2.0, PrintFeatureType::Infill, true);
    output();
    output_.str("");
    gcode.writeExtrusion(Point3LL(2000, 0, 0), { 1.0, 1000.0, 20.0 }, 2.0, PrintFeatureType::Infill, true);
    EXPECT_DOUBLE_EQ(gcode.getCurrentExtrudedVolume(), 4.0);
    EXPECT_EQ(output().str(), ";FLOW_RATE_COMPENSATED_OFFSET = 1.000000\nG1 F60 X2 Y0.00\n");
    output_.str("");
    gcode.writeTravel(Point3LL(3000, 0, 0), { 1.0, 1000.0, 20.0 });
    gcode.writeExtrusion(Point3LL(4000, 0, 0), { 1.0, 1000.0, 20.0 }, 2.0, PrintFeatureType::Infill, true);
    EXPECT_EQ(output().str(), "G0 X3 Y0.00\nG1 X4 Y0.00 E4\n");
    EXPECT_DOUBLE_EQ(gcode.getCurrentExtrudedVolume(), 6.0);
}

TEST_F(GCodeExportTest, RelativeCompensationCancellationPreservesLogicalExtrusion)
{
    prepareMotion();
    stationaryPrime(0.0, true);
    gcode.setFlowRateExtrusionSettings(10.0, 1.0);
    gcode.writeExtrusion(Point3LL(1000, 0, 0), { 2.0, 1000.0, 20.0 }, 2.0, PrintFeatureType::Infill, true);
    output();
    output_.str("");
    gcode.writeExtrusion(Point3LL(2000, 0, 0), { 1.0, 1000.0, 20.0 }, 2.0, PrintFeatureType::Infill, true);
    EXPECT_DOUBLE_EQ(gcode.getCurrentExtrudedVolume(), 4.0);
    EXPECT_EQ(output().str(), ";FLOW_RATE_COMPENSATED_OFFSET = 1.000000\nG1 F60 X2 Y0.00\n");
    output_.str("");
    gcode.writeExtrusion(Point3LL(3000, 0, 0), { 1.0, 1000.0, 20.0 }, 2.0, PrintFeatureType::Infill, true);
    EXPECT_EQ(output().str(), "G1 X3 Y0.00 E1\n");
}

TEST_F(GCodeExportTest, ResetRebasesCompensationAndEstimator)
{
    checkCompensationReset(false, false);
}

TEST_F(GCodeExportTest, RelativeResetRebasesCompensationAndEstimator)
{
    checkCompensationReset(true, false);
}

TEST_F(GCodeExportTest, VolumetricResetRebasesCompensationAndEstimator)
{
    checkCompensationReset(false, true);
}

TEST_F(GCodeExportTest, RelativeVolumetricResetRebasesCompensationAndEstimator)
{
    checkCompensationReset(true, true);
}

TEST_F(GCodeExportTest, ToolSwitchRebasesCompensationAndEstimator)
{
    prepareMotion();
    auto& scene = Application::getInstance().current_slice_->scene;
    scene.extruders.emplace_back(1, &scene.settings);
    for (auto& train : scene.extruders)
    {
        train.settings_.add("machine_extruder_prestart_code", "");
        train.settings_.add("machine_extruder_start_code", "");
        train.settings_.add("machine_extruder_end_code", "");
        train.settings_.add("machine_extruder_start_code_duration", "0");
        train.settings_.add("machine_extruder_end_code_duration", "0");
        train.settings_.add("machine_extruder_change_duration", "0");
        train.settings_.add("retraction_enable", "false");
    }
    refreshMotionSettings();
    gcode.setFilamentDiameter(1, 2000);
    gcode.setFlowRateExtrusionSettings(10.0, 1.0);
    gcode.writeExtrusion(Point3LL(1000, 0, 0), { 2.0, 1000.0, 20.0 }, 2.0, PrintFeatureType::Infill, true);
    output();
    output_.str("");
    RetractionConfig config{};
    config.distance = 1.0;
    config.speed = 50.0;
    gcode.switchExtruder(1, config);
    gcode.writeRetraction(config, true);
    EXPECT_EQ(output().str(), "G92 E0\nT1\nG92 E0\nG1 F3000 E-1\n");
    EXPECT_DOUBLE_EQ(gcode.getTotalFilamentUsed(0), 2.0);
}

TEST_F(GCodeExportTest, TravelFlagsSelectIndependentDerivatives)
{
    prepareMotion();
    motionSettings().add("acceleration_travel_enabled", "false");
    motionSettings().add("jerk_travel_enabled", "true");
    refreshMotionSettings();
    const SpeedDerivatives first = gcode.getTravelDerivatives(motionExtruder());
    EXPECT_EQ(first.acceleration, 500.0);
    EXPECT_EQ(first.jerk, 20.0);
    motionSettings().add("acceleration_travel_enabled", "true");
    motionSettings().add("jerk_travel_enabled", "false");
    refreshMotionSettings();
    const SpeedDerivatives second = gcode.getTravelDerivatives(motionExtruder());
    EXPECT_EQ(second.acceleration, 1000.0);
    EXPECT_EQ(second.jerk, 10.0);
}

TEST_F(GCodeExportTest, SpecialTravelPreservesSelectedLayerAndDestinationDerivatives)
{
    prepareMotion();
    motionSettings().add("acceleration_travel_enabled", "false");
    motionSettings().add("jerk_travel_enabled", "false");
    refreshMotionSettings();
    gcode.setTravelDerivatives({ 80.0, 430.0, 7.3 });
    const SpeedDerivatives derivatives = gcode.getTravelDerivatives(motionExtruder(), 12.0);
    EXPECT_EQ(derivatives.speed, 12.0);
    EXPECT_EQ(derivatives.acceleration, 430.0);
    EXPECT_EQ(derivatives.jerk, 7.3);
}

TEST_F(GCodeExportTest, RepRapAndCheetahAxisJerkUnits)
{
    prepareMotion();
    motionSettings().add("jerk_enabled", "true");
    refreshMotionSettings();
    selectFlavor(EGCodeFlavor::REPRAP);
    gcode.writeTravel(Point3LL(0, 0, 1000), { 10.0, 100.0, 0.3 });
    EXPECT_NE(output().str().find("M566 Z18 E300"), std::string::npos);
    output_.str("");
    selectFlavor(EGCodeFlavor::CHEETAH);
    gcode.writeTravel(Point3LL(0, 0, 2000), { 10.0, 100.0, 0.2 });
    EXPECT_NE(output().str().find("M215 Z200 E5000"), std::string::npos);
}

TEST_F(GCodeExportTest, FirmwareRetractionRemainsAMacro)
{
    prepareMotion();
    useFirmwareRetraction();
    RetractionConfig config{};
    config.distance = 2.0;
    config.speed = 100.0;
    gcode.writeRetraction(config, true);
    EXPECT_EQ(output().str(), "G10\n");
}

TEST_F(GCodeExportTest, PostPrimeZCorrectionHasExplicitBoundedFeedrate)
{
    prepareMotion();
    selectFlavor(EGCodeFlavor::GRIFFIN);
    motionExtruder().settings_.add("prime_blob_enable", "false");
    motionSettings().add("machine_max_feedrate_z", "5");
    refreshMotionSettings();
    gcode.writePrimeTrain({ 100.0, 1000.0, 20.0 });
    EXPECT_EQ(output().str(), "G280 S1\nG0 F300 Z0.00\n");
}

TEST_F(GCodeExportTest, BFBFeedrateIsCapped)
{
    prepareMotion(4.0);
    selectFlavor(EGCodeFlavor::BFB);
    motionSettings().add("machine_max_feedrate_y", "30");
    refreshMotionSettings();
    gcode.writeTravel(Point3LL(1000, 0, 0), { 100.0, 1000.0, 20.0 });
    EXPECT_NE(output().str().find("G1 X1 Y0.00 Z0.00 F1800"), std::string::npos);
}

TEST_F(GCodeExportTest, SplitHopRetractionAndPrimingUseMachineELimits)
{
    prepareMotion();
    motionSettings().add("material_max_flowrate", "0");
    motionSettings().add("machine_max_feedrate_e", "5");
    refreshMotionSettings();
    gcode.writeZhopStart(1000, SpeedDerivatives{ 100.0, 1000.0, 20.0 }, 10.0, 0.5);
    gcode.writeZhopEnd(SpeedDerivatives{ 100.0, 1000.0, 20.0 }, 0, 0.0, 0.5);
    const std::string result = output().str();
    EXPECT_NE(result.find("G1 F15 Z0.5 E-10"), std::string::npos);
    EXPECT_NE(result.find("G1 F2400 Z1"), std::string::npos);
    EXPECT_NE(result.find("G1 F15 Z0.00 E0"), std::string::npos);
}

TEST_F(GCodeExportTest, StationaryPrimingUsesOnlyMachineEDerivatives)
{
    prepareMotion();
    motionSettings().add("acceleration_enabled", "true");
    motionSettings().add("jerk_enabled", "true");
    motionSettings().add("machine_max_feedrate_e", "5");
    motionSettings().add("machine_max_acceleration_e", "300");
    motionSettings().add("machine_max_jerk_e", "0.5");
    refreshMotionSettings();
    stationaryPrime(4.0);
    const std::string result = output().str();
    EXPECT_NE(result.find("M201 Z100 E300"), std::string::npos);
    EXPECT_NE(result.find("M204 R300"), std::string::npos);
    EXPECT_NE(result.find("M205 Z0.4 E0.5"), std::string::npos);
    EXPECT_EQ(result.find("M205 X"), std::string::npos);
    EXPECT_NE(result.find("G1 F300 E2"), std::string::npos);
}

TEST_F(GCodeExportTest, ManufacturerCodeIsUnchangedAndInvalidatesMotionCache)
{
    prepareMotion();
    motionSettings().add("acceleration_enabled", "true");
    motionSettings().add("jerk_enabled", "true");
    refreshMotionSettings();
    gcode.writeTravel(Point3LL(1000, 0, 0), { 30.0, 700.0, 12.0 });
    output();
    output_.str("");
    const std::string manufacturer_code = "M204 S9000\nM205 X99\nG1 F9999\n";
    gcode.writeResolvableGCode(manufacturer_code, 0, {});
    gcode.writeTravel(Point3LL(2000, 0, 0), { 30.0, 700.0, 12.0 });
    const std::string result = output().str();
    EXPECT_EQ(result.substr(0, manufacturer_code.size()), manufacturer_code);
    EXPECT_NE(result.find("M204 S700"), std::string::npos);
    EXPECT_NE(result.find("M205 X12 Y12"), std::string::npos);
    EXPECT_NE(result.find("G0 F1800 X2 Y0.00"), std::string::npos);
}

TEST_F(GCodeExportTest, RepRapStationaryPrimingRestoresPrintAndTravelModes)
{
    prepareMotion();
    selectFlavor(EGCodeFlavor::REPRAP);
    motionSettings().add("acceleration_enabled", "true");
    refreshMotionSettings();
    gcode.writeTravel(Point3LL(1000, 0, 0), { 30.0, 700.0, 12.0 });
    stationaryPrime(4.0);
    gcode.writeExtrusion(Point3LL(2000, 0, 0), { 30.0, 500.0, 10.0 }, 1.0, PrintFeatureType::Infill);
    const std::string result = output().str();
    EXPECT_NE(result.find("M204 T700"), std::string::npos);
    EXPECT_NE(result.find("M204 P10000"), std::string::npos);
    EXPECT_NE(result.find("M204 P500"), std::string::npos);
}

TEST_F(GCodeExportTest, BFBPrintingUsesMaterialCapAndCommandPrecision)
{
    prepareMotion(4.0);
    selectFlavor(EGCodeFlavor::BFB);
    motionSettings().add("material_max_flowrate", "1");
    refreshMotionSettings();
    gcode.writeTravel(Point3LL(1000, 0, 0), { 100.0, 1000.0, 20.0 });
    gcode.writeExtrusion(Point3LL(11000, 0, 0), { 100.0, 1000.0, 20.0 }, 0.4, PrintFeatureType::Infill);
    const std::string result = output().str();
    EXPECT_NE(result.find("M108 S3.8"), std::string::npos);
    EXPECT_NE(result.find("M101"), std::string::npos);
    EXPECT_EQ(gcode.getCurrentSpeed(), 2.5);
}

TEST_F(GCodeExportTest, AccelerationCommandsUseOriginalIntegerPrecision)
{
    prepareMotion();
    motionSettings().add("acceleration_enabled", "true");
    motionSettings().add("machine_max_acceleration_e", "0.1");
    refreshMotionSettings();
    gcode.writeExtrusion(Point3LL(1000, 0, 0), { 100.0, 1000.0, 20.0 }, 4.0, PrintFeatureType::Infill);
    EXPECT_NE(output().str().find("M204 S0\n"), std::string::npos);
    EXPECT_NE(output().str().find("M201 Z100 E0\n"), std::string::npos);
}

TEST_F(GCodeExportTest, LegacyFlavorsDoNotReceiveUnsupportedMotionCommands)
{
    prepareMotion();
    motionSettings().add("acceleration_enabled", "true");
    motionSettings().add("jerk_enabled", "true");
    motionSettings().add("machine_max_feedrate_y", "30");
    refreshMotionSettings();
    coord_t x = 1000;
    for (const EGCodeFlavor flavor : { EGCodeFlavor::MAKERBOT, EGCodeFlavor::MACH3 })
    {
        output_.str("");
        selectFlavor(flavor);
        gcode.writeTravel(Point3LL(x, 0, 0), { 100.0, 1000.0, 20.0 });
        EXPECT_NE(output().str().find("G0 F1800"), std::string::npos);
        EXPECT_EQ(output().str().find("M201"), std::string::npos);
        EXPECT_EQ(output().str().find("M204"), std::string::npos);
        EXPECT_EQ(output().str().find("M205"), std::string::npos);
        x += 1000;
    }
}

TEST_F(GCodeExportTest, MotionCachesLoadAfterConstructionAndRefreshForSequentialGroups)
{
    addPreSetupSettings();
    const Settings loaded_settings = motionSettings();
    Application::getInstance().current_slice_.reset();
    GCodeExport fresh;
    EXPECT_EQ(fresh.getCurrentSpeed(), 1.0);

    Application::getInstance().current_slice_ = std::make_shared<Slice>(2);
    Scene& scene = Application::getInstance().current_slice_->scene;
    scene.extruders.emplace_back(0, &scene.mesh_groups[0].settings);
    scene.extruders.emplace_back(1, &scene.mesh_groups[0].settings);
    EXPECT_THROW(fresh.getTravelDerivatives(scene.extruders[0]), std::bad_optional_access);
    // Like the CLI/Arcus loaders, construct the exporter and extruders before supplying settings.
    scene.settings = loaded_settings;
    scene.settings.add("machine_max_feedrate_y", "30");
    scene.extruders[0].settings_.add("speed_travel", "80");
    scene.extruders[1].settings_.add("speed_travel", "60");
    scene.extruders[0].settings_.add("acceleration_travel", "222");
    scene.extruders[0].settings_.add("jerk_travel", "6");
    scene.extruders[0].settings_.add("material_max_flowrate", "2");
    scene.extruders[1].settings_.add("material_max_flowrate", "4");
    fresh.preSetup(0);
    EXPECT_EQ(fresh.getCurrentSpeed(), -1.0);
    EXPECT_EQ(fresh.getTravelDerivatives(scene.extruders[0]).speed, 80.0);
    EXPECT_EQ(fresh.getTravelDerivatives(scene.extruders[1]).speed, 60.0);
    EXPECT_EQ(fresh.getTravelDerivatives(scene.extruders[0]).acceleration, 222.0);
    EXPECT_EQ(fresh.getTravelDerivatives(scene.extruders[0]).jerk, 6.0);
    fresh.writeTravel(Point3LL(1000, 0, 0), { 100.0, 1000.0, 20.0 });
    EXPECT_EQ(fresh.getCurrentSpeed(), 30.0);
    fresh.writeExtrusion(Point3LL(2000, 0, 0), { 100.0, 1000.0, 20.0 }, 1.0, PrintFeatureType::Infill);
    EXPECT_EQ(fresh.getCurrentSpeed(), 2.0);

    RetractionConfig no_retraction{};
    fresh.setTravelDerivatives({ 10.0, 123.0, 4.0 });
    fresh.switchExtruder(1, no_retraction);
    EXPECT_EQ(fresh.getTravelDerivatives(scene.extruders[1]).acceleration, 123.0);
    fresh.writeExtrusion(Point3LL(3000, 0, 0), { 100.0, 1000.0, 20.0 }, 1.0, PrintFeatureType::Infill);
    EXPECT_EQ(fresh.getCurrentSpeed(), 4.0);
    fresh.setTravelDerivatives({ 10.0, 123.0, 4.0 });

    scene.current_mesh_group = scene.mesh_groups.begin() + 1;
    scene.current_mesh_group->settings.add("machine_max_feedrate_y", "12");
    scene.current_mesh_group->settings.add("acceleration_travel", "345");
    for (ExtruderTrain& extruder : scene.extruders)
    {
        extruder.settings_.setParent(&scene.current_mesh_group->settings);
    }
    fresh.preSetup(1);
    EXPECT_EQ(fresh.getCurrentSpeed(), -1.0);
    EXPECT_EQ(fresh.getTravelDerivatives(scene.extruders[1]).acceleration, 345.0);
    EXPECT_EQ(fresh.getTravelDerivatives(scene.extruders[1]).speed, 60.0);
    EXPECT_EQ(fresh.getTravelDerivatives(scene.extruders[0]).acceleration, 222.0);
    EXPECT_EQ(fresh.getTravelDerivatives(scene.extruders[0]).jerk, 6.0);
    fresh.writeTravel(Point3LL(4000, 0, 0), { 100.0, 1000.0, 20.0 });
    EXPECT_EQ(fresh.getCurrentSpeed(), 12.0);
    fresh.writeExtrusion(Point3LL(5000, 0, 0), { 100.0, 1000.0, 20.0 }, 1.0, PrintFeatureType::Infill);
    EXPECT_EQ(fresh.getCurrentSpeed(), 4.0);
}

TEST_F(GCodeExportTest, DerivativesKeepFullPrecisionUntilCommandFormatting)
{
    prepareMotion();
    motionSettings().add("acceleration_enabled", "true");
    motionSettings().add("jerk_enabled", "true");
    motionSettings().add("machine_max_feedrate_y", "30.1234");
    refreshMotionSettings();
    gcode.writeTravel(Point3LL(1000, 0, 0), { 100.0, 700.456, 12.346 });
    EXPECT_DOUBLE_EQ(gcode.getCurrentSpeed(), 30.1234);
    EXPECT_NE(output().str().find("M204 S700\n"), std::string::npos);
    EXPECT_NE(output().str().find("M205 X12.35 Y12.35\n"), std::string::npos);
}

TEST_F(GCodeExportTest, FinalMotionResetRestoresDefaultsWithinMachineLimits)
{
    prepareMotion();
    motionSettings().add("acceleration_enabled", "true");
    motionSettings().add("jerk_enabled", "true");
    motionSettings().add("machine_max_acceleration_y", "700");
    refreshMotionSettings();
    selectFlavor(EGCodeFlavor::REPRAP);
    gcode.writeTravel(Point3LL(0, 0, 1000), { 10.0, 5.0, 0.1 });
    output();
    output_.str("");
    gcode.resetMotionSettings();
    const std::string result = output().str();
    EXPECT_NE(result.find("M201 Z100 E10000"), std::string::npos);
    EXPECT_NE(result.find("M204 P700"), std::string::npos);
    EXPECT_NE(result.find("M204 T700"), std::string::npos);
    EXPECT_NE(result.find("M566 X1200 Y1200"), std::string::npos);
}

TEST_F(GCodeExportTest, FinalMotionResetRespectsControlEnableFlags)
{
    prepareMotion();
    gcode.resetMotionSettings();
    EXPECT_TRUE(output().str().empty());
    motionSettings().add("jerk_enabled", "true");
    refreshMotionSettings();
    gcode.resetMotionSettings();
    EXPECT_EQ(output().str().find("M204"), std::string::npos);
    EXPECT_NE(output().str().find("M205 X20 Y20"), std::string::npos);
}

TEST_F(GCodeExportTest, MeshGroupControlOverridesGateHeadAxisAndFinalResetConsistently)
{
    prepareMotion();
    Settings& group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;
    group_settings.add("acceleration_enabled", "true");
    group_settings.add("jerk_enabled", "true");
    EXPECT_FALSE(motionSettings().get<bool>("acceleration_enabled"));
    EXPECT_FALSE(motionSettings().get<bool>("jerk_enabled"));
    refreshMotionSettings();
    selectFlavor(EGCodeFlavor::REPRAP);
    gcode.writeTravel(Point3LL(1000, 0, 0), { 30.0, 700.0, 12.0 });
    gcode.writeExtrusion(Point3LL(2000, 0, 0), { 30.0, 500.0, 10.0 }, 1.0, PrintFeatureType::Infill);
    const std::string moves = output().str();
    EXPECT_NE(moves.find("M201 Z100 E10000"), std::string::npos);
    EXPECT_NE(moves.find("M204 T700"), std::string::npos);
    EXPECT_NE(moves.find("M204 P500"), std::string::npos);
    EXPECT_NE(moves.find("M566 X600 Y600"), std::string::npos);
    output_.str("");
    gcode.resetMotionSettings();
    const std::string reset = output().str();
    EXPECT_NE(reset.find("M204 P1000"), std::string::npos);
    EXPECT_NE(reset.find("M204 T1000"), std::string::npos);
    EXPECT_NE(reset.find("M566 X1200 Y1200"), std::string::npos);

    motionSettings().add("acceleration_enabled", "true");
    motionSettings().add("jerk_enabled", "true");
    group_settings.add("acceleration_enabled", "false");
    group_settings.add("jerk_enabled", "false");
    refreshMotionSettings();
    output_.str("");
    gcode.writeTravel(Point3LL(3000, 0, 0), { 30.0, 700.0, 12.0 });
    gcode.resetMotionSettings();
    const std::string disabled = output().str();
    EXPECT_EQ(disabled.find("M201"), std::string::npos);
    EXPECT_EQ(disabled.find("M204"), std::string::npos);
    EXPECT_EQ(disabled.find("M566"), std::string::npos);
}

TEST_F(GCodeExportTest, ReplacingSpeedPreservesOtherDerivatives)
{
    const SpeedDerivatives defaults;
    EXPECT_EQ(defaults.speed, 0.0);
    EXPECT_EQ(defaults.acceleration, 0.0);
    EXPECT_EQ(defaults.jerk, 0.0);
    const SpeedDerivatives original{ 100.0, 430.5, 7.3 };
    const SpeedDerivatives changed(original, 12.0);
    EXPECT_EQ(changed.speed, 12.0);
    EXPECT_EQ(changed.acceleration, original.acceleration);
    EXPECT_EQ(changed.jerk, original.jerk);
    EXPECT_EQ(original.speed, 100.0);
}

} // namespace cura
// NOLINTEND(*-magic-numbers)
