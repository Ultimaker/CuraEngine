// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "arachne/SkeletalTrapezoidation.h"

#include <memory>

#include <gtest/gtest.h>

#include "BeadingStrategy/BeadingStrategyFactory.h"
#include "geometry/MendedShape.h"
#include "geometry/Point2LL.h"
#include "geometry/Shape.h"
#include "settings/Settings.h"
#include "utils/Coord_t.h"
#include "utils/section_type.h"

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{

/*!
 * Exposes the protected central-edge filter and replaces the Voronoi graph with a hand-built one.
 *
 * The base constructor always builds a Voronoi diagram, and an empty diagram crashes, so the probe
 * is constructed from a plain square and then the generated graph is discarded.
 */
class FilterCentralProbe : public SkeletalTrapezoidation
{
public:
    FilterCentralProbe(const BeadingStrategy& strategy, const Settings& settings, const Shape& outline)
        : SkeletalTrapezoidation(MendedShape(&settings, SectionType::WALL, &outline), strategy, AngleRadians(0.5), 200, 1000, 0, 400, 0, SectionType::WALL)
    {
        graph_.edges_.clear();
        graph_.nodes_.clear();
    }

    using SkeletalTrapezoidation::filterCentral;

    STHalfEdgeNode* addNode(const Point2LL& point, const coord_t radius)
    {
        graph_.nodes_.emplace_back(SkeletalTrapezoidationJoint(), point);
        STHalfEdgeNode* node = &graph_.nodes_.back();
        node->data_.distance_to_boundary_ = radius;
        return node;
    }

    /*!
     * Add a twin pair. The returned edge points from \p from to \p to.
     */
    STHalfEdge* addEdge(STHalfEdgeNode* from, STHalfEdgeNode* to, const bool central)
    {
        graph_.edges_.emplace_back(SkeletalTrapezoidationEdge());
        STHalfEdge* forward = &graph_.edges_.back();
        graph_.edges_.emplace_back(SkeletalTrapezoidationEdge());
        STHalfEdge* backward = &graph_.edges_.back();

        forward->twin_ = backward;
        backward->twin_ = forward;
        forward->from_ = from;
        forward->to_ = to;
        backward->from_ = to;
        backward->to_ = from;
        forward->data_.setIsCentral(central);
        backward->data_.setIsCentral(central);
        if (from->incident_edge_ == nullptr)
        {
            from->incident_edge_ = forward;
        }
        if (to->incident_edge_ == nullptr)
        {
            to->incident_edge_ = backward;
        }
        return forward;
    }
};

class FilterCentralTest : public testing::Test
{
public:
    Settings settings_;
    Shape outline_;
    BeadingStrategyPtr strategy_{ BeadingStrategyFactory::makeStrategy() };
    std::unique_ptr<FilterCentralProbe> probe_;

    FilterCentralTest()
    {
        // Same mending settings WallsComputationTest uses for a plain square, so the throwaway
        // Voronoi input survives outline repair and the constructor can finish.
        settings_.add("fill_outline_gaps", "false");
        settings_.add("meshfix_maximum_deviation", "0.1");
        settings_.add("meshfix_maximum_extrusion_area_deviation", "0.01");
        settings_.add("meshfix_fluid_motion_enabled", "false");
        settings_.add("meshfix_maximum_resolution", "0.01");
        settings_.add("min_wall_line_width", "0.3");
        settings_.add("min_feature_size", "0");
        settings_.add("wall_line_width_0", "0.4");

        outline_.emplace_back();
        outline_.back().emplace_back(0, 0);
        outline_.back().emplace_back(MM2INT(20), 0);
        outline_.back().emplace_back(MM2INT(20), MM2INT(20));
        outline_.back().emplace_back(0, MM2INT(20));

        probe_ = std::make_unique<FilterCentralProbe>(*strategy_, settings_, outline_);
    }

    static bool isCentral(const STHalfEdge* edge)
    {
        return edge->data_.isCentral();
    }
};

TEST_F(FilterCentralTest, DissolvesShortCentralEdgeThatIsNotALocalMaximum)
{
    // Radii differ so this is an ordinary sloping end. Equal radii would walk a null next_ inside canGoUp.
    STHalfEdgeNode* start = probe_->addNode(Point2LL(0, 0), 80);
    STHalfEdgeNode* end = probe_->addNode(Point2LL(10, 0), 100);
    STHalfEdge* edge = probe_->addEdge(start, end, true);
    ASSERT_FALSE(start->isLocalMaximum(true));
    ASSERT_FALSE(end->isLocalMaximum(true));

    probe_->filterCentral(20);

    EXPECT_FALSE(isCentral(edge));
    EXPECT_FALSE(isCentral(edge->twin_));
}

TEST_F(FilterCentralTest, KeepsCentralEdgeLongerThanTheFilterDistance)
{
    STHalfEdgeNode* start = probe_->addNode(Point2LL(0, 0), 80);
    STHalfEdgeNode* end = probe_->addNode(Point2LL(100, 0), 100);
    STHalfEdge* edge = probe_->addEdge(start, end, true);
    ASSERT_FALSE(start->isLocalMaximum(true));
    ASSERT_FALSE(end->isLocalMaximum(true));

    probe_->filterCentral(20);

    EXPECT_TRUE(isCentral(edge));
    EXPECT_TRUE(isCentral(edge->twin_));
}

TEST_F(FilterCentralTest, KeepsShortCentralEdgeThatArrivesAtAStrictLocalMaximum)
{
    // The spoke is not central, so the only central edge runs from a boundary tip into an interior local maximum.
    STHalfEdgeNode* tip = probe_->addNode(Point2LL(0, 0), 100);
    STHalfEdgeNode* peak = probe_->addNode(Point2LL(10, 0), 300);
    STHalfEdgeNode* side = probe_->addNode(Point2LL(10, 10), 50);

    STHalfEdge* tip_to_peak = probe_->addEdge(tip, peak, true);
    STHalfEdge* peak_to_side = probe_->addEdge(peak, side, false);
    tip_to_peak->next_ = peak_to_side;
    peak_to_side->prev_ = tip_to_peak;
    peak_to_side->twin_->next_ = tip_to_peak->twin_;
    tip_to_peak->twin_->prev_ = peak_to_side->twin_;
    peak->incident_edge_ = tip_to_peak->twin_;

    ASSERT_TRUE(peak->isLocalMaximum(true));
    ASSERT_FALSE(tip->isLocalMaximum(true));

    probe_->filterCentral(20);

    EXPECT_TRUE(isCentral(tip_to_peak));
    EXPECT_TRUE(isCentral(tip_to_peak->twin_));
}

} // namespace cura
// NOLINTEND(*-magic-numbers)
