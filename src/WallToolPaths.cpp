// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "WallToolPaths.h"

#include <algorithm> //For std::partition_copy and std::min_element.
#include <unordered_set>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>
#include <scripta/logger.h>

#include "ExtruderTrain.h"
#include "SkeletalTrapezoidation.h"
#include "utils/ExtrusionLineStitcher.h"
#include "utils/Simplify.h"
#include "utils/SparsePointGrid.h" //To stitch the inner contour.
#include "utils/actions/smooth.h"
#include "utils/polygonUtils.h"

namespace cura
{

WallToolPaths::WallToolPaths(
    const Shape& outline,
    const coord_t nominal_bead_width,
    const size_t inset_count,
    const coord_t wall_0_inset,
    const Settings& settings,
    const int layer_idx,
    SectionType section_type)
    : outline_(outline)
    , bead_width_0_(nominal_bead_width)
    , bead_width_x_(nominal_bead_width)
    , inset_count_(inset_count)
    , wall_0_inset_(wall_0_inset)
    , print_thin_walls_(settings.get<bool>("fill_outline_gaps"))
    , min_feature_size_(settings.get<coord_t>("min_feature_size"))
    , min_bead_width_(settings.get<coord_t>("min_bead_width"))
    , small_area_length_(INT2MM(static_cast<double>(nominal_bead_width) / 2))
    , toolpaths_generated_(false)
    , settings_(settings)
    , layer_idx_(layer_idx)
    , section_type_(section_type)
{
}

WallToolPaths::WallToolPaths(
    const Shape& outline,
    const coord_t bead_width_0,
    const coord_t bead_width_x,
    const size_t inset_count,
    const coord_t wall_0_inset,
    const Settings& settings,
    const int layer_idx,
    SectionType section_type)
    : outline_(outline)
    , bead_width_0_(bead_width_0)
    , bead_width_x_(bead_width_x)
    , inset_count_(inset_count)
    , wall_0_inset_(wall_0_inset)
    , print_thin_walls_(settings.get<bool>("fill_outline_gaps"))
    , min_feature_size_(settings.get<coord_t>("min_feature_size"))
    , min_bead_width_(settings.get<coord_t>("min_bead_width"))
    , small_area_length_(INT2MM(static_cast<double>(bead_width_0) / 2))
    , toolpaths_generated_(false)
    , settings_(settings)
    , layer_idx_(layer_idx)
    , section_type_(section_type)
{
}

const std::vector<VariableWidthLines>& WallToolPaths::generate()
{
    const coord_t allowed_distance = settings_.get<coord_t>("meshfix_maximum_deviation");

    // Sometimes small slivers of polygons mess up the prepared_outline. By performing an open-close operation
    // with half the minimum printable feature size or minimum line width, these slivers are removed, while still
    // keeping enough information to not degrade the print quality;
    // These features can't be printed anyhow. See PR CuraEngine#1811 for some screenshots
    const coord_t open_close_distance
        = settings_.get<bool>("fill_outline_gaps") ? settings_.get<coord_t>("min_feature_size") / 2 - 5 : settings_.get<coord_t>("min_wall_line_width") / 2 - 5;
    const coord_t epsilon_offset = (allowed_distance / 2) - 1;
    const auto transitioning_angle = settings_.get<AngleRadians>("wall_transition_angle");
    constexpr coord_t discretization_step_size = MM2INT(0.8);

    // Simplify outline for boost::voronoi consumption. Absolutely no self intersections or near-self intersections allowed:
    // TODO: Open question: Does this indeed fix all (or all-but-one-in-a-million) cases for manifold but otherwise possibly complex polygons?
    Shape prepared_outline = outline_.offset(-open_close_distance).offset(open_close_distance * 2).offset(-open_close_distance);
    scripta::log("prepared_outline_0", prepared_outline, section_type_, layer_idx_);
    prepared_outline.removeSmallAreas(small_area_length_ * small_area_length_, false);
    prepared_outline = Simplify(settings_).polygon(prepared_outline);
    if (settings_.get<bool>("meshfix_fluid_motion_enabled") && section_type_ != SectionType::SUPPORT)
    {
        // No need to smooth support walls
        auto smoother = actions::smooth(settings_);
        for (Polygon& polygon : prepared_outline)
        {
            polygon.setPoints(smoother(polygon.getPoints()));
        }
    }

    PolygonUtils::fixSelfIntersections(epsilon_offset, prepared_outline);
    prepared_outline.removeDegenerateVerts();
    prepared_outline.removeColinearEdges(AngleRadians(0.005));
    // Removing collinear edges may introduce self intersections, so we need to fix them again
    PolygonUtils::fixSelfIntersections(epsilon_offset, prepared_outline);
    prepared_outline.removeDegenerateVerts();
    prepared_outline = prepared_outline.unionPolygons();
    prepared_outline = Simplify(settings_).polygon(prepared_outline);

    if (prepared_outline.area() <= 0)
    {
        assert(toolpaths_.empty());
        return toolpaths_;
    }

    prepared_outline = prepared_outline.removeNearSelfIntersections();

    const coord_t wall_transition_length = settings_.get<coord_t>("wall_transition_length");

    // When to split the middle wall into two:
    const double min_even_wall_line_width = settings_.get<double>("min_even_wall_line_width");
    const double wall_line_width_0 = settings_.get<double>("wall_line_width_0");
    const Ratio wall_split_middle_threshold = std::max(1.0, std::min(99.0, 100.0 * (2.0 * min_even_wall_line_width - wall_line_width_0) / wall_line_width_0)) / 100.0;

    // When to add a new middle in between the innermost two walls:
    const double min_odd_wall_line_width = settings_.get<double>("min_odd_wall_line_width");
    const double wall_line_width_x = settings_.get<double>("wall_line_width_x");
    const Ratio wall_add_middle_threshold = std::max(1.0, std::min(99.0, 100.0 * min_odd_wall_line_width / wall_line_width_x)) / 100.0;

    const int wall_distribution_count = settings_.get<int>("wall_distribution_count");
    const size_t max_bead_count = (inset_count_ < std::numeric_limits<size_t>::max() / 2) ? 2 * inset_count_ : std::numeric_limits<size_t>::max();
    const auto beading_strat = BeadingStrategyFactory::makeStrategy(
        bead_width_0_,
        bead_width_x_,
        wall_transition_length,
        transitioning_angle,
        print_thin_walls_,
        min_bead_width_,
        min_feature_size_,
        wall_split_middle_threshold,
        wall_add_middle_threshold,
        max_bead_count,
        wall_0_inset_,
        wall_distribution_count);
    const auto transition_filter_dist = settings_.get<coord_t>("wall_transition_filter_distance");
    const auto allowed_filter_deviation = settings_.get<coord_t>("wall_transition_filter_deviation");
    SkeletalTrapezoidation wall_maker(
        prepared_outline,
        *beading_strat,
        beading_strat->getTransitioningAngle(),
        discretization_step_size,
        transition_filter_dist,
        allowed_filter_deviation,
        wall_transition_length,
        layer_idx_,
        section_type_);
    wall_maker.generateToolpaths(toolpaths_);
    scripta::log(
        "toolpaths_0",
        toolpaths_,
        section_type_,
        layer_idx_,
        scripta::CellVDI{ "is_closed", &ExtrusionLine::is_closed_ },
        scripta::CellVDI{ "is_odd", &ExtrusionLine::is_odd_ },
        scripta::CellVDI{ "inset_idx", &ExtrusionLine::inset_idx_ },
        scripta::PointVDI{ "width", &ExtrusionJunction::w_ },
        scripta::PointVDI{ "perimeter_index", &ExtrusionJunction::perimeter_index_ });

    stitchToolPaths(toolpaths_, settings_);
    scripta::log(
        "toolpaths_1",
        toolpaths_,
        section_type_,
        layer_idx_,
        scripta::CellVDI{ "is_closed", &ExtrusionLine::is_closed_ },
        scripta::CellVDI{ "is_odd", &ExtrusionLine::is_odd_ },
        scripta::CellVDI{ "inset_idx", &ExtrusionLine::inset_idx_ },
        scripta::PointVDI{ "width", &ExtrusionJunction::w_ },
        scripta::PointVDI{ "perimeter_index", &ExtrusionJunction::perimeter_index_ });

    removeSmallFillLines(toolpaths_);
    scripta::log(
        "toolpaths_2",
        toolpaths_,
        section_type_,
        layer_idx_,
        scripta::CellVDI{ "is_closed", &ExtrusionLine::is_closed_ },
        scripta::CellVDI{ "is_odd", &ExtrusionLine::is_odd_ },
        scripta::CellVDI{ "inset_idx", &ExtrusionLine::inset_idx_ },
        scripta::PointVDI{ "width", &ExtrusionJunction::w_ },
        scripta::PointVDI{ "perimeter_index", &ExtrusionJunction::perimeter_index_ });

    simplifyToolPaths(toolpaths_, settings_);
    scripta::log(
        "toolpaths_3",
        toolpaths_,
        section_type_,
        layer_idx_,
        scripta::CellVDI{ "is_closed", &ExtrusionLine::is_closed_ },
        scripta::CellVDI{ "is_odd", &ExtrusionLine::is_odd_ },
        scripta::CellVDI{ "inset_idx", &ExtrusionLine::inset_idx_ },
        scripta::PointVDI{ "width", &ExtrusionJunction::w_ },
        scripta::PointVDI{ "perimeter_index", &ExtrusionJunction::perimeter_index_ });

    separateOutInnerContour();

    removeEmptyToolPaths(toolpaths_);
    scripta::log(
        "toolpaths_4",
        toolpaths_,
        section_type_,
        layer_idx_,
        scripta::CellVDI{ "is_closed", &ExtrusionLine::is_closed_ },
        scripta::CellVDI{ "is_odd", &ExtrusionLine::is_odd_ },
        scripta::CellVDI{ "inset_idx", &ExtrusionLine::inset_idx_ },
        scripta::PointVDI{ "width", &ExtrusionJunction::w_ },
        scripta::PointVDI{ "perimeter_index", &ExtrusionJunction::perimeter_index_ });
    assert(
        std::is_sorted(
            toolpaths_.cbegin(),
            toolpaths_.cend(),
            [](const VariableWidthLines& l, const VariableWidthLines& r)
            {
                return l.front().inset_idx_ < r.front().inset_idx_;
            })
        && "WallToolPaths should be sorted from the outer 0th to inner_walls");
    toolpaths_generated_ = true;
    scripta::log(
        "toolpaths_5",
        toolpaths_,
        section_type_,
        layer_idx_,
        scripta::CellVDI{ "is_closed", &ExtrusionLine::is_closed_ },
        scripta::CellVDI{ "is_odd", &ExtrusionLine::is_odd_ },
        scripta::CellVDI{ "inset_idx", &ExtrusionLine::inset_idx_ },
        scripta::PointVDI{ "width", &ExtrusionJunction::w_ },
        scripta::PointVDI{ "perimeter_index", &ExtrusionJunction::perimeter_index_ });
    return toolpaths_;
}


void WallToolPaths::stitchToolPaths(std::vector<VariableWidthLines>& toolpaths, const Settings& settings)
{
    const coord_t stitch_distance
        = settings.get<coord_t>("wall_line_width_x") - 1; // In 0-width contours, junctions can cause up to 1-line-width gaps. Don't stitch more than 1 line width.

    for (unsigned int wall_idx = 0; wall_idx < toolpaths.size(); wall_idx++)
    {
        VariableWidthLines& wall_lines = toolpaths[wall_idx];

        VariableWidthLines stitched_polylines;
        VariableWidthLines closed_polygons;
        ExtrusionLineStitcher::stitch(wall_lines, stitched_polylines, closed_polygons, stitch_distance);
        wall_lines = stitched_polylines; // replace input toolpaths with stitched polylines

        for (ExtrusionLine& wall_polygon : closed_polygons)
        {
            if (wall_polygon.junctions_.empty())
            {
                continue;
            }
            wall_polygon.is_closed_ = true;
            wall_lines.emplace_back(std::move(wall_polygon)); // add stitched polygons to result
        }
#ifdef DEBUG
        for (ExtrusionLine& line : wall_lines)
        {
            assert(line.inset_idx_ == wall_idx);
        }
#endif // DEBUG
    }
}

void WallToolPaths::removeSmallFillLines(std::vector<VariableWidthLines>& toolpaths)
{
    for (VariableWidthLines& inset : toolpaths)
    {
        for (size_t line_idx = 0; line_idx < inset.size(); line_idx++)
        {
            ExtrusionLine& line = inset[line_idx];
            if (line.is_outer_wall())
            {
                continue;
            }
            coord_t min_width = std::numeric_limits<coord_t>::max();
            for (const ExtrusionJunction& j : line)
            {
                min_width = std::min(min_width, j.w_);
            }
            if (line.is_odd_ && ! line.is_closed_ && line.shorterThan(min_width / 2))
            { // remove line
                line = std::move(inset.back());
                inset.erase(--inset.end());
                line_idx--; // reconsider the current position
            }
        }
    }
}

void WallToolPaths::simplifyToolPaths(std::vector<VariableWidthLines>& toolpaths, const Settings& settings)
{
    const Simplify simplifier(settings);
    for (auto& toolpath : toolpaths)
    {
        toolpath = toolpath
                 | ranges::views::transform(
                       [&simplifier](auto& line)
                       {
                           auto line_ = line.is_closed_ ? simplifier.polygon(line) : simplifier.polyline(line);

                           if (line_.is_closed_ && line_.size() >= 2 && line_.front() != line_.back())
                           {
                               line_.emplace_back(line_.front());
                           }
                           return line_;
                       })
                 | ranges::views::filter(
                       [](const auto& line)
                       {
                           return ! line.empty();
                       })
                 | ranges::to_vector;
    }
}

const std::vector<VariableWidthLines>& WallToolPaths::getToolPaths()
{
    if (! toolpaths_generated_)
    {
        return generate();
    }
    return toolpaths_;
}

void WallToolPaths::pushToolPaths(std::vector<VariableWidthLines>& paths)
{
    if (! toolpaths_generated_)
    {
        generate();
    }
    paths.insert(paths.end(), toolpaths_.begin(), toolpaths_.end());
}

void WallToolPaths::separateOutInnerContour()
{
    // We'll remove all 0-width paths from the original toolpaths and store them separately as polygons.
    std::vector<VariableWidthLines> actual_toolpaths;
    actual_toolpaths.reserve(toolpaths_.size()); // A bit too much, but the correct order of magnitude.
    std::vector<VariableWidthLines> contour_paths;
    contour_paths.reserve(toolpaths_.size() / inset_count_);
    inner_contour_.clear();
    for (const VariableWidthLines& inset : toolpaths_)
    {
        if (inset.empty())
        {
            continue;
        }
        bool is_contour = false;
        for (const ExtrusionLine& line : inset)
        {
            for (const ExtrusionJunction& j : line)
            {
                if (j.w_ == 0)
                {
                    is_contour = true;
                }
                else
                {
                    is_contour = false;
                }
                break;
            }
        }


        if (is_contour)
        {
#ifdef DEBUG
            for (const ExtrusionLine& line : inset)
            {
                for (const ExtrusionJunction& j : line)
                {
                    assert(j.w_ == 0);
                }
            }
#endif // DEBUG
            for (const ExtrusionLine& line : inset)
            {
                if (line.is_odd_)
                {
                    continue; // odd lines don't contribute to the contour
                }
                else if (line.is_closed_) // sometimes an very small even polygonal wall is not stitched into a polygon
                {
                    inner_contour_.emplace_back(line.toPolygon());
                }
            }
        }
        else
        {
            actual_toolpaths.emplace_back(inset);
        }
    }
    if (! actual_toolpaths.empty())
    {
        toolpaths_ = std::move(actual_toolpaths); // Filtered out the 0-width paths.
    }
    else
    {
        toolpaths_.clear();
    }

    // The output walls from the skeletal trapezoidation have no known winding order, especially if they are joined together from polylines.
    // They can be in any direction, clockwise or counter-clockwise, regardless of whether the shapes are positive or negative.
    // To get a correct shape, we need to make the outside contour positive and any holes inside negative.
    // This can be done by applying the even-odd rule to the shape. This rule is not sensitive to the winding order of the polygon.
    // The even-odd rule would be incorrect if the polygon self-intersects, but that should never be generated by the skeletal trapezoidation.
    inner_contour_ = inner_contour_.processEvenOdd();
}

const Shape& WallToolPaths::getInnerContour()
{
    if (! toolpaths_generated_ && inset_count_ > 0)
    {
        generate();
    }
    else if (inset_count_ == 0)
    {
        return outline_;
    }
    return inner_contour_;
}

bool WallToolPaths::removeEmptyToolPaths(std::vector<VariableWidthLines>& toolpaths)
{
    for (VariableWidthLines& toolpath : toolpaths)
    {
        toolpath.erase(
            std::remove_if(
                toolpath.begin(),
                toolpath.end(),
                [](const ExtrusionLine& line)
                {
                    return line.junctions_.empty();
                }),
            toolpath.end());
    }

    toolpaths.erase(
        std::remove_if(
            toolpaths.begin(),
            toolpaths.end(),
            [](const VariableWidthLines& lines)
            {
                return lines.empty();
            }),
        toolpaths.end());
    return toolpaths.empty();
}

} // namespace cura
