// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <range/v3/view/split.hpp>

#include <gtest/gtest.h>

#include "ExtruderTrain.h"
#include "LayerPlan.h"

namespace cura
{

struct RetractionBaseConfig
{
    Ratio during_travel;
    Velocity speed;
};

struct AntiOozeAmountsTestDataSet
{
    struct
    {
        bool firmware_retract;
        EGCodeFlavor flavor;
    } printer_capacities;

    struct
    {
        int speed;
        coord_t height;
    } z_hop;

    struct
    {
        std::vector<Point3LL> path;
        Velocity speed;
    } travel;

    struct
    {
        RetractionBaseConfig config;
        double distance;
    } retraction;

    struct
    {
        RetractionBaseConfig config;
        double extra_prime_volume;
    } prime;
};

struct AntiOozeAmountsTestResult
{
    TravelAntiOozing retraction;
    TravelAntiOozing priming;
};

struct AntiOozeAmountsTestCase
{
    AntiOozeAmountsTestDataSet data;
    AntiOozeAmountsTestResult result;
};

class AntiOozeAmountsTest : public testing::TestWithParam<AntiOozeAmountsTestCase>
{
public:
    void SetUp() override
    {
        const AntiOozeAmountsTestDataSet& data = GetParam().data;

        gcode_.extruder_attr_[0].machine_firmware_retract_ = data.printer_capacities.firmware_retract;
        gcode_.extruder_attr_[0].filament_area_ = 1.0; // Makes manual calculations simpler...
        gcode_.flavor_ = data.printer_capacities.flavor;
        gcode_.current_position_ = data.travel.path.front();

        extruder_.settings_.add("speed_z_hop", std::to_string(data.z_hop.speed));

        path_.points = ranges::to<std::vector>(data.travel.path | ranges::views::drop(1));
        path_.config.speed_derivatives.speed = data.travel.speed;
        path_.perform_z_hop = data.z_hop.height > 0;

        RetractionConfig retraction_config;
        retraction_config.distance = data.retraction.distance;
        retraction_config.retract_during_travel = data.retraction.config.during_travel;
        retraction_config.prime_during_travel = data.prime.config.during_travel;
        retraction_config.speed = data.retraction.config.speed;
        retraction_config.primeSpeed = data.prime.config.speed;
        retraction_config.prime_volume = data.prime.extra_prime_volume;
        retraction_config_.retraction_config = retraction_config;
    }

    /*!
     * Cleaning up after a test is hardly necessary but just for neatness.
     */
    void TearDown() override
    {
    }

protected:
    GCodeExport gcode_;
    ExtruderTrain extruder_{ 0, nullptr };
    GCodePath path_;
    RetractionAndWipeConfig retraction_config_;
};

TEST_P(AntiOozeAmountsTest, ComputeAntiOozeAmounts)
{
    const AntiOozeAmountsTestCase& test_case = GetParam();
    const AntiOozeAmountsTestDataSet& data = test_case.data;
    const AntiOozeAmountsTestResult& result = test_case.result;

    std::optional<TravelAntiOozing> retraction_amounts;
    std::optional<TravelAntiOozing> priming_amounts;

    LayerPlan::computeAntiOozeAmounts(gcode_, extruder_, path_, data.z_hop.height, &retraction_config_, retraction_amounts, priming_amounts);

    constexpr double threshold = 0.001; // We don't need a very low threshold, we just want to make sure the values are consistent

    EXPECT_TRUE(retraction_amounts.has_value());
    EXPECT_NEAR(retraction_amounts->amount_while_still, result.retraction.amount_while_still, threshold);
    EXPECT_NEAR(retraction_amounts->z_hop.amount, result.retraction.z_hop.amount, threshold);
    EXPECT_NEAR(retraction_amounts->z_hop.ratio, result.retraction.z_hop.ratio, threshold);
    EXPECT_NEAR(retraction_amounts->amount_while_travel, result.retraction.amount_while_travel, threshold);
    EXPECT_EQ(retraction_amounts->segment_split_position, result.retraction.segment_split_position);
    EXPECT_EQ(retraction_amounts->amount_by_segment.size(), result.retraction.amount_by_segment.size());
    for (size_t i = 0; i < retraction_amounts->amount_by_segment.size() && i < result.retraction.amount_by_segment.size(); ++i)
    {
        EXPECT_NEAR(retraction_amounts->amount_by_segment.at(i), result.retraction.amount_by_segment.at(i), threshold);
    }

    EXPECT_TRUE(priming_amounts.has_value());
    EXPECT_NEAR(priming_amounts->amount_while_still, result.priming.amount_while_still, threshold);
    EXPECT_NEAR(priming_amounts->z_hop.amount, result.priming.z_hop.amount, threshold);
    EXPECT_NEAR(priming_amounts->z_hop.ratio, result.priming.z_hop.ratio, threshold);
    EXPECT_NEAR(priming_amounts->amount_while_travel, result.priming.amount_while_travel, threshold);
    EXPECT_EQ(priming_amounts->segment_split_position, result.priming.segment_split_position);
    EXPECT_EQ(priming_amounts->amount_by_segment.size(), result.priming.amount_by_segment.size());
    for (size_t i = 0; i < priming_amounts->amount_by_segment.size() && i < result.priming.amount_by_segment.size(); ++i)
    {
        EXPECT_NEAR(priming_amounts->amount_by_segment.at(i), result.priming.amount_by_segment.at(i), threshold);
    }
}

std::vector<AntiOozeAmountsTestCase> GetTestCases()
{
    std::vector<AntiOozeAmountsTestCase> data_sets;

    std::vector path_simple = { Point3LL(0, 0, 0), Point3LL(100000, 0, 0) }; // Straight 10cm path on X

    // Single segment, all retraction/prime during stationary
    data_sets.push_back(
        AntiOozeAmountsTestCase{ .data
                                 = AntiOozeAmountsTestDataSet{ .printer_capacities = { .firmware_retract = false, .flavor = EGCodeFlavor::MARLIN },
                                                               .z_hop = { .speed = 50, .height = 1000 },
                                                               .travel = { .path = path_simple, .speed = 150.0 },
                                                               .retraction = { .config = RetractionBaseConfig{ .during_travel = 0.0_r, .speed = 50.0 }, .distance = 5.0 },
                                                               .prime = { .config = RetractionBaseConfig{ .during_travel = 0.0_r, .speed = 50.0 }, .extra_prime_volume = 0.0 } },
                                 .result = AntiOozeAmountsTestResult{ .retraction = TravelAntiOozing{ .amount_while_still = 5.0,
                                                                                                      .z_hop = ZHopAntiOozing{ .amount = 5.0, .ratio = 0.0_r },
                                                                                                      .amount_while_travel = 5.0,
                                                                                                      .segment_split_position = Point2LL(0, 0),
                                                                                                      .amount_by_segment = { 5.0 } },
                                                                      .priming = TravelAntiOozing{ .amount_while_still = 5.0,
                                                                                                   .z_hop = ZHopAntiOozing{ .amount = 5.0, .ratio = 0.0_r },
                                                                                                   .amount_while_travel = 5.0,
                                                                                                   .segment_split_position = Point2LL(100000, 0),
                                                                                                   .amount_by_segment = { 5.0 } } } });

    // Single segment, all retraction/prime during travel, no z-hop
    data_sets.push_back(
        AntiOozeAmountsTestCase{ .data
                                 = AntiOozeAmountsTestDataSet{ .printer_capacities = { .firmware_retract = false, .flavor = EGCodeFlavor::MARLIN },
                                                               .z_hop = { .speed = 50, .height = 0 },
                                                               .travel = { .path = path_simple, .speed = 100.0 },
                                                               .retraction = { .config = RetractionBaseConfig{ .during_travel = 1.0_r, .speed = 10.0 }, .distance = 2.0 },
                                                               .prime = { .config = RetractionBaseConfig{ .during_travel = 1.0_r, .speed = 10.0 }, .extra_prime_volume = 0.0 } },
                                 .result = AntiOozeAmountsTestResult{ .retraction = TravelAntiOozing{ .amount_while_still = 0.0,
                                                                                                      .z_hop = ZHopAntiOozing{ .amount = 0.0, .ratio = 0.0_r },
                                                                                                      .amount_while_travel = 2.0,
                                                                                                      .segment_split_position = Point2LL(20000, 0),
                                                                                                      .amount_by_segment = { 2.0 } },
                                                                      .priming = TravelAntiOozing{ .amount_while_still = 0.0,
                                                                                                   .z_hop = ZHopAntiOozing{ .amount = 0.0, .ratio = 0.0_r },
                                                                                                   .amount_while_travel = 2.0,
                                                                                                   .segment_split_position = Point2LL(80000, 0),
                                                                                                   .amount_by_segment = { 0.0 } } } });

    // Single segment, retraction/prime 50% during travel, no z-hop
    data_sets.push_back(
        AntiOozeAmountsTestCase{ .data
                                 = AntiOozeAmountsTestDataSet{ .printer_capacities = { .firmware_retract = false, .flavor = EGCodeFlavor::MARLIN },
                                                               .z_hop = { .speed = 50, .height = 0 },
                                                               .travel = { .path = path_simple, .speed = 100.0 },
                                                               .retraction = { .config = RetractionBaseConfig{ .during_travel = 0.5_r, .speed = 10.0 }, .distance = 2.0 },
                                                               .prime = { .config = RetractionBaseConfig{ .during_travel = 0.5_r, .speed = 10.0 }, .extra_prime_volume = 0.0 } },
                                 .result = AntiOozeAmountsTestResult{ .retraction = TravelAntiOozing{ .amount_while_still = 1.0,
                                                                                                      .z_hop = ZHopAntiOozing{ .amount = 1.0, .ratio = 0.0_r },
                                                                                                      .amount_while_travel = 2.0,
                                                                                                      .segment_split_position = Point2LL(10000, 0),
                                                                                                      .amount_by_segment = { 2.0 } },
                                                                      .priming = TravelAntiOozing{ .amount_while_still = 1.0,
                                                                                                   .z_hop = ZHopAntiOozing{ .amount = 1.0, .ratio = 0.0_r },
                                                                                                   .amount_while_travel = 2.0,
                                                                                                   .segment_split_position = Point2LL(90000, 0),
                                                                                                   .amount_by_segment = { 1.0 } } } });

    // Single segment, retraction 50% during travel, prime 100 %during stationary, with z-hop
    data_sets.push_back(
        AntiOozeAmountsTestCase{ .data
                                 = AntiOozeAmountsTestDataSet{ .printer_capacities = { .firmware_retract = false, .flavor = EGCodeFlavor::MARLIN },
                                                               .z_hop = { .speed = 50, .height = 1000 },
                                                               .travel = { .path = path_simple, .speed = 100.0 },
                                                               .retraction = { .config = RetractionBaseConfig{ .during_travel = 0.5_r, .speed = 10.0 }, .distance = 2.0 },
                                                               .prime = { .config = RetractionBaseConfig{ .during_travel = 0.0_r, .speed = 10.0 }, .extra_prime_volume = 0.0 } },
                                 .result = AntiOozeAmountsTestResult{ .retraction = TravelAntiOozing{ .amount_while_still = 1.0,
                                                                                                      .z_hop = ZHopAntiOozing{ .amount = 1.2, .ratio = 1.0_r },
                                                                                                      .amount_while_travel = 2.0,
                                                                                                      .segment_split_position = Point2LL(8000, 0),
                                                                                                      .amount_by_segment = { 2.0 } },
                                                                      .priming = TravelAntiOozing{ .amount_while_still = 2.0,
                                                                                                   .z_hop = ZHopAntiOozing{ .amount = 2.0, .ratio = 0.0_r },
                                                                                                   .amount_while_travel = 2.0,
                                                                                                   .segment_split_position = Point2LL(100000, 0),
                                                                                                   .amount_by_segment = { 2.0 } } } });

    // Single segment, retraction 100% during stationary, prime 50 %during travel, with z-hop
    data_sets.push_back(
        AntiOozeAmountsTestCase{ .data
                                 = AntiOozeAmountsTestDataSet{ .printer_capacities = { .firmware_retract = false, .flavor = EGCodeFlavor::MARLIN },
                                                               .z_hop = { .speed = 50, .height = 1000 },
                                                               .travel = { .path = path_simple, .speed = 100.0 },
                                                               .retraction = { .config = RetractionBaseConfig{ .during_travel = 0.0_r, .speed = 10.0 }, .distance = 2.0 },
                                                               .prime = { .config = RetractionBaseConfig{ .during_travel = 0.5_r, .speed = 10.0 }, .extra_prime_volume = 0.0 } },
                                 .result = AntiOozeAmountsTestResult{ .retraction = TravelAntiOozing{ .amount_while_still = 2.0,
                                                                                                      .z_hop = ZHopAntiOozing{ .amount = 2.0, .ratio = 0.0_r },
                                                                                                      .amount_while_travel = 2.0,
                                                                                                      .segment_split_position = Point2LL(0, 0),
                                                                                                      .amount_by_segment = { 2.0 } },
                                                                      .priming = TravelAntiOozing{ .amount_while_still = 1.0,
                                                                                                   .z_hop = ZHopAntiOozing{ .amount = 1.2, .ratio = 1.0_r },
                                                                                                   .amount_while_travel = 2.0,
                                                                                                   .segment_split_position = Point2LL(92000, 0),
                                                                                                   .amount_by_segment = { 1.2 } } } });

    // 10cm corner path, split by 1cm segments
    std::vector path_corner
        = { Point3LL(0, 0, 0),         Point3LL(10000, 0, 0),     Point3LL(20000, 0, 0),     Point3LL(30000, 0, 0),     Point3LL(40000, 0, 0),    Point3LL(50000, 0, 0),
            Point3LL(50000, 10000, 0), Point3LL(50000, 20000, 0), Point3LL(50000, 30000, 0), Point3LL(50000, 40000, 0), Point3LL(50000, 50000, 0) };

    // Multi segment, all retraction/prime during stationary
    data_sets.push_back(
        AntiOozeAmountsTestCase{ .data
                                 = AntiOozeAmountsTestDataSet{ .printer_capacities = { .firmware_retract = false, .flavor = EGCodeFlavor::MARLIN },
                                                               .z_hop = { .speed = 50, .height = 1000 },
                                                               .travel = { .path = path_corner, .speed = 150.0 },
                                                               .retraction = { .config = RetractionBaseConfig{ .during_travel = 0.0_r, .speed = 50.0 }, .distance = 5.0 },
                                                               .prime = { .config = RetractionBaseConfig{ .during_travel = 0.0_r, .speed = 50.0 }, .extra_prime_volume = 0.0 } },
                                 .result = AntiOozeAmountsTestResult{ .retraction = TravelAntiOozing{ .amount_while_still = 5.0,
                                                                                                      .z_hop = ZHopAntiOozing{ .amount = 5.0, .ratio = 0.0_r },
                                                                                                      .amount_while_travel = 5.0,
                                                                                                      .segment_split_position = Point2LL(0, 0),
                                                                                                      .amount_by_segment = { 5.0 } },
                                                                      .priming = TravelAntiOozing{ .amount_while_still = 5.0,
                                                                                                   .z_hop = ZHopAntiOozing{ .amount = 5.0, .ratio = 0.0_r },
                                                                                                   .amount_while_travel = 5.0,
                                                                                                   .segment_split_position = Point2LL(50000, 50000),
                                                                                                   .amount_by_segment = { 5.0 } } } });

    // Multi segment, all retraction/prime during travel, no z-hop
    data_sets.push_back(
        AntiOozeAmountsTestCase{ .data
                                 = AntiOozeAmountsTestDataSet{ .printer_capacities = { .firmware_retract = false, .flavor = EGCodeFlavor::MARLIN },
                                                               .z_hop = { .speed = 50, .height = 0 },
                                                               .travel = { .path = path_corner, .speed = 100.0 },
                                                               .retraction = { .config = RetractionBaseConfig{ .during_travel = 1.0_r, .speed = 10.0 }, .distance = 2.0 },
                                                               .prime = { .config = RetractionBaseConfig{ .during_travel = 1.0_r, .speed = 10.0 }, .extra_prime_volume = 0.0 } },
                                 .result = AntiOozeAmountsTestResult{ .retraction = TravelAntiOozing{ .amount_while_still = 0.0,
                                                                                                      .z_hop = ZHopAntiOozing{ .amount = 0.0, .ratio = 0.0_r },
                                                                                                      .amount_while_travel = 2.0,
                                                                                                      .segment_split_position = Point2LL(20000, 0),
                                                                                                      .amount_by_segment = { 1.0, 2.0 } },
                                                                      .priming = TravelAntiOozing{ .amount_while_still = 0.0,
                                                                                                   .z_hop = ZHopAntiOozing{ .amount = 0.0, .ratio = 0.0_r },
                                                                                                   .amount_while_travel = 2.0,
                                                                                                   .segment_split_position = Point2LL(50000, 30000),
                                                                                                   .amount_by_segment = { 0.0, 1.0 } } } });

    // Multi segment, all retraction/prime during travel, with z-hop
    data_sets.push_back(
        AntiOozeAmountsTestCase{ .data
                                 = AntiOozeAmountsTestDataSet{ .printer_capacities = { .firmware_retract = false, .flavor = EGCodeFlavor::MARLIN },
                                                               .z_hop = { .speed = 50, .height = 1000 },
                                                               .travel = { .path = path_corner, .speed = 100.0 },
                                                               .retraction = { .config = RetractionBaseConfig{ .during_travel = 1.0_r, .speed = 10.0 }, .distance = 2.0 },
                                                               .prime = { .config = RetractionBaseConfig{ .during_travel = 1.0_r, .speed = 10.0 }, .extra_prime_volume = 0.0 } },
                                 .result = AntiOozeAmountsTestResult{ .retraction = TravelAntiOozing{ .amount_while_still = 0.0,
                                                                                                      .z_hop = ZHopAntiOozing{ .amount = 0.2, .ratio = 1.0_r },
                                                                                                      .amount_while_travel = 2.0,
                                                                                                      .segment_split_position = Point2LL(18000, 0),
                                                                                                      .amount_by_segment = { 1.2, 2.0 } },
                                                                      .priming = TravelAntiOozing{ .amount_while_still = 0.0,
                                                                                                   .z_hop = ZHopAntiOozing{ .amount = 0.2, .ratio = 1.0_r },
                                                                                                   .amount_while_travel = 2.0,
                                                                                                   .segment_split_position = Point2LL(50000, 32000),
                                                                                                   .amount_by_segment = { 0.2, 1.2 } } } });

    // Multi segment, all retraction/prime during travel, extra stationary time, with z-hop
    data_sets.push_back(
        AntiOozeAmountsTestCase{ .data
                                 = AntiOozeAmountsTestDataSet{ .printer_capacities = { .firmware_retract = false, .flavor = EGCodeFlavor::MARLIN },
                                                               .z_hop = { .speed = 50, .height = 1000 },
                                                               .travel = { .path = path_corner, .speed = 100.0 },
                                                               .retraction = { .config = RetractionBaseConfig{ .during_travel = 1.0_r, .speed = 10.0 }, .distance = 10.0 },
                                                               .prime = { .config = RetractionBaseConfig{ .during_travel = 1.0_r, .speed = 10.0 }, .extra_prime_volume = 0.0 } },
                                 .result = AntiOozeAmountsTestResult{ .retraction = TravelAntiOozing{ .amount_while_still = 4.8,
                                                                                                      .z_hop = ZHopAntiOozing{ .amount = 5.0, .ratio = 1.0_r },
                                                                                                      .amount_while_travel = 10.0,
                                                                                                      .segment_split_position = Point2LL(50000, 0),
                                                                                                      .amount_by_segment = { 6.0, 7.0, 8.0, 9.0, 10.0 } },
                                                                      .priming = TravelAntiOozing{ .amount_while_still = 4.8,
                                                                                                   .z_hop = ZHopAntiOozing{ .amount = 5.0, .ratio = 1.0_r },
                                                                                                   .amount_while_travel = 10.0,
                                                                                                   .segment_split_position = Point2LL(50000, 0),
                                                                                                   .amount_by_segment = { 5.0, 6.0, 7.0, 8.0, 9.0 } } } });

    // Worst-case scenario: multi segment, all retraction/prime during travel, extra stationary time, with z-hop, with extra prime volume
    data_sets.push_back(
        AntiOozeAmountsTestCase{
            .data = AntiOozeAmountsTestDataSet{ .printer_capacities = { .firmware_retract = false, .flavor = EGCodeFlavor::MARLIN },
                                                .z_hop = { .speed = 50, .height = 1000 },
                                                .travel = { .path = path_corner, .speed = 100.0 },
                                                .retraction = { .config = RetractionBaseConfig{ .during_travel = 1.0_r, .speed = 10.0 }, .distance = 10.0 },
                                                .prime = { .config = RetractionBaseConfig{ .during_travel = 1.0_r, .speed = 10.0 }, .extra_prime_volume = 5.0 } },
            .result = AntiOozeAmountsTestResult{ .retraction = TravelAntiOozing{ .amount_while_still = 5.816,
                                                                                 .z_hop = ZHopAntiOozing{ .amount = 6.016, .ratio = 1.0_r },
                                                                                 .amount_while_travel = 10.0,
                                                                                 .segment_split_position = Point2LL(39837, 0),
                                                                                 .amount_by_segment = { 7.016, 8.016, 9.016, 10.0 } },
                                                 .priming = TravelAntiOozing{ .amount_while_still = 8.783,
                                                                              .z_hop = ZHopAntiOozing{ .amount = 8.983, .ratio = 1.0_r },
                                                                              .amount_while_travel = 15.0,
                                                                              .segment_split_position = Point2LL(39837, 0),
                                                                              .amount_by_segment = { 8.983, 9.983, 10.983, 11.983, 12.983, 13.983, 14.983 } } } });

    return data_sets;
}

INSTANTIATE_TEST_SUITE_P(MeaningfulTestParameters, AntiOozeAmountsTest, testing::ValuesIn(GetTestCases()));

} // namespace cura
