// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "GcodeTemplateResolver.h"

#include <gtest/gtest.h>

#include "Application.h"
#include "Slice.h"

struct GCodeTemplateTestCase
{
    std::string input;
    std::optional<int> extruder_nr;
    std::string expected_output;
    std::unordered_map<std::string, std::string> extra_settings{};
};

class GCodeTemplateResolverTest : public ::testing::TestWithParam<GCodeTemplateTestCase>
{
protected:
    void SetUp() override
    {
        std::shared_ptr<cura::Slice> slice = std::make_shared<cura::Slice>(0);
        cura::Application::getInstance().current_slice_ = slice;

        slice->scene.settings.add("bed_temperature", "50.0");
        slice->scene.settings.add("initial_extruder", "0");

        slice->scene.extruders.emplace_back(0, &slice->scene.settings);
        slice->scene.extruders[0].settings_.add("material_temperature", "190.5");

        slice->scene.extruders.emplace_back(1, &slice->scene.settings);
        slice->scene.extruders[1].settings_.add("material_temperature", "210.0");
    }

    void TearDown() override
    {
    }
};

TEST_P(GCodeTemplateResolverTest, ResolveTemplate)
{
    const auto& param = GetParam();
    std::string result = cura::GcodeTemplateResolver::resolveGCodeTemplate(param.input, param.extruder_nr, param.extra_settings);
    EXPECT_EQ(result, param.expected_output);
}

INSTANTIATE_TEST_SUITE_P(
    GCodeTemplateCases,
    GCodeTemplateResolverTest,
    ::testing::Values(
        // Static code
        GCodeTemplateTestCase{ "G0", std::nullopt, "G0\n" },
        // Basic replacement
        GCodeTemplateTestCase{ "M128 {bed_temperature}", std::nullopt, "M128 50\n" },
        // Conditional expression with global setting
        GCodeTemplateTestCase{ "{if bed_temperature > 30}\n"
                               "G123\n"
                               "{else}\n"
                               "G456\n"
                               "{endif}",
                               std::nullopt,
                               "G123\n" },
        // Conditional expression with extruder setting directly specified by index 0
        GCodeTemplateTestCase{ "{if material_temperature > 200, 0}\n"
                               "G10\n"
                               "{else}\n"
                               "G20\n"
                               "{endif}",
                               std::nullopt,
                               "G20\n" },
        // Conditional expression with extruder setting directly specified by index 1
        GCodeTemplateTestCase{ "{if material_temperature > 200, 1}\n"
                               "G100\n"
                               "{else}\n"
                               "G200\n"
                               "{endif}",
                               std::nullopt,
                               "G100\n" },
        // Conditional expression with extruder specified by context (0)
        GCodeTemplateTestCase{ "{if material_temperature > 200}\n"
                               "C10\n"
                               "{else}\n"
                               "C20\n"
                               "{endif}",
                               0,
                               "C20\n" },
        // Conditional expression with extruder specified by context (1)
        GCodeTemplateTestCase{ "{if material_temperature > 200}\n"
                               "C100\n"
                               "{else}\n"
                               "C200\n"
                               "{endif}",
                               1,
                               "C100\n" },
        // Conditional expression with extruder index specified by setting
        GCodeTemplateTestCase{ "{if material_temperature > 200, initial_extruder}\n"
                               "G1000\n"
                               "{else}\n"
                               "G2000\n"
                               "{endif}",
                               std::nullopt,
                               "G2000\n" },
        // Conditional expression with extruder index specified by formula
        GCodeTemplateTestCase{ "{if material_temperature > 200, (initial_extruder + 1) % 2}\n"
                               "X1000\n"
                               "{else}\n"
                               "X2000\n"
                               "{endif}",
                               std::nullopt,
                               "X1000\n" },
        // Conditional expression with elsif
        GCodeTemplateTestCase{ "{if bed_temperature < 30}\n"
                               "T30\n"
                               "{elif bed_temperature >= 30 and bed_temperature < 40}\n"
                               "T40\n"
                               "{elif bed_temperature >= 40 and bed_temperature < 50}\n"
                               "T50\n"
                               "{elif bed_temperature >= 50 and bed_temperature < 60}\n"
                               "T60\n"
                               "{elif bed_temperature >= 60 and bed_temperature < 70}\n"
                               "T70\n"
                               "{else}\n"
                               "T-800\n"
                               "{endif}",
                               std::nullopt,
                               "T60\n" },
        // Formula inside a conditional expression
        GCodeTemplateTestCase{ "{if bed_temperature < 30}\n"
                               "Z000\n"
                               "{else}\n"
                               "Z{bed_temperature + 10}\n"
                               "{endif}",
                               std::nullopt,
                               "Z60\n" },
        // Other commands around conditional expression
        GCodeTemplateTestCase{ "R000\n"
                               "# My super initial command\n"
                               "R111 X123 Y456 Z789\n"
                               "{if bed_temperature > 30}\n"
                               "R987\n"
                               "R654 X321\n"
                               "{else}\n"
                               "R963 X852 Y741\n"
                               "R321 X654 Y987\n"
                               "{endif}\n"
                               "# And finally, the end of the start at the beginning of the header\n"
                               "R369\n"
                               "R357 X951 Y843",
                               std::nullopt,
                               "R000\n"
                               "# My super initial command\n"
                               "R111 X123 Y456 Z789\n"
                               "R987\n"
                               "R654 X321\n"
                               "# And finally, the end of the start at the beginning of the header\n"
                               "R369\n"
                               "R357 X951 Y843\n" },
        // Multiple conditional expressions
        GCodeTemplateTestCase{ "A999\n"
                               "{if bed_temperature > 30}\n"
                               "A000\n"
                               "{else}\n"
                               "A100\n"
                               "{endif}\n"
                               "A888\n"
                               "{if material_temperature > 200, 0}\n"
                               "A200\n"
                               "{else}\n"
                               "A300\n"
                               "{endif}\n"
                               "A777\n",
                               std::nullopt,
                               "A999\n"
                               "A000\n"
                               "A888\n"
                               "A300\n"
                               "A777\n" },
        // Nested condition expression
        GCodeTemplateTestCase{ "{if bed_temperature < 30}\n"
                               "{if material_temperature < 30, 0}\n"
                               "M000\n"
                               "{else}\n"
                               "M888\n"
                               "{endif}\n"
                               "{else}\n"
                               "M{bed_temperature + 10}\n"
                               "{endif}",
                               std::nullopt,
                               "{if bed_temperature < 30}\n"
                               "{if material_temperature < 30, 0}\n"
                               "M000\n"
                               "{else}\n"
                               "M888\n"
                               "{endif}\n"
                               "{else}\n"
                               "M{bed_temperature + 10}\n"
                               "{endif}\n" },
        // Wrong condition expression
        GCodeTemplateTestCase{ "{of material_temperature > 200, 1}\n"
                               "G100\n"
                               "{else}\n"
                               "G200\n"
                               "{endif}",
                               std::nullopt,
                               "{of material_temperature > 200, 1}\n"
                               "G100\n"
                               "{else}\n"
                               "G200\n"
                               "{endif}\n" },
        // Condition expression without start (should raise error, but here we expect empty output)
        GCodeTemplateTestCase{ "W100\n"
                               "{else}\n"
                               "W200\n"
                               "{endif}",
                               std::nullopt,
                               "W100\n"
                               "{else}\n"
                               "W200\n"
                               "{endif}\n" },
        // Formula with non-existing variable
        GCodeTemplateTestCase{ "{material_storage_temperature}", std::nullopt, "{material_storage_temperature}\n" },
        // Missing formula end character
        GCodeTemplateTestCase{ "{material_temperature, 0", std::nullopt, "{material_temperature, 0\n" },
        // Conditional expression with missing end character
        GCodeTemplateTestCase{ "{if material_temperature > 200, 0\n"
                               "Q1000\n"
                               "{else}\n"
                               "Q2000\n"
                               "{endif}",
                               std::nullopt,
                               "{if material_temperature > 200, 0\n"
                               "Q1000\n"
                               "{else}\n"
                               "Q2000\n"
                               "{endif}\n" },
        // Unexpected end character
        GCodeTemplateTestCase{ "{if material_temperature > 200, 0}}\n"
                               "S1000\n"
                               "{else}}\n"
                               "S2000\n"
                               "{endif}",
                               std::nullopt,
                               "}\n"
                               "S2000\n" },
        // Multiple replaces on single line
        GCodeTemplateTestCase{ "BT={bed_temperature} IE={initial_extruder}", std::nullopt, "BT=50 IE=0\n" },
        // Multiple extruder replaces on single line
        GCodeTemplateTestCase{ "MT0={material_temperature, 0} MT1={material_temperature, 1}", std::nullopt, "MT0=190.5 MT1=210\n" },
        // Extra settings
        GCodeTemplateTestCase{ "ES={bed_temperature} SE={max_printer_temperature}", std::nullopt, "ES=50 SE=500\n", { { "max_printer_temperature", "500" } } }));