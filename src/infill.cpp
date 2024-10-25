// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "infill.h"

#include <algorithm> //For std::sort.
#include <functional>
#include <numbers>
#include <unordered_set>

#include <scripta/logger.h>
#include <spdlog/spdlog.h>

#include "WallToolPaths.h"
#include "geometry/OpenPolyline.h"
#include "geometry/PointMatrix.h"
#include "infill/GyroidInfill.h"
#include "infill/ImageBasedDensityProvider.h"
#include "infill/LightningGenerator.h"
#include "infill/NoZigZagConnectorProcessor.h"
#include "infill/SierpinskiFill.h"
#include "infill/SierpinskiFillProvider.h"
#include "infill/SubDivCube.h"
#include "infill/UniformDensityProvider.h"
#include "plugins/slots.h"
#include "sliceDataStorage.h"
#include "utils/OpenPolylineStitcher.h"
#include "utils/PolygonConnector.h"
#include "utils/Simplify.h"
#include "utils/UnionFind.h"
#include "utils/linearAlg2D.h"
#include "utils/polygonUtils.h"

/*!
 * Function which returns the scanline_idx for a given x coordinate
 *
 * For negative \p x this is different from simple division.
 *
 * \warning \p line_width is assumed to be positive
 *
 * \param x the point to get the scansegment index for
 * \param line_width the width of the scan segments
 */
static inline int computeScanSegmentIdx(int x, int line_width)
{
    if (x < 0)
    {
        return (x + 1) / line_width - 1;
        // - 1 because -1 belongs to scansegment -1
        // + 1 because -line_width belongs to scansegment -1
    }
    return x / line_width;
}

namespace cura
{

Shape Infill::generateWallToolPaths(
    std::vector<VariableWidthLines>& toolpaths,
    const Shape& outer_contour,
    const size_t wall_line_count,
    const coord_t line_width,
    const Settings& settings,
    int layer_idx,
    SectionType section_type)
{
    Shape inner_contour;
    if (wall_line_count > 0)
    {
        constexpr coord_t wall_0_inset = 0; // Don't apply any outer wall inset for these. That's just for the outer wall.
        WallToolPaths wall_toolpaths(outer_contour, line_width, wall_line_count, wall_0_inset, settings, layer_idx, section_type);
        wall_toolpaths.pushToolPaths(toolpaths);
        inner_contour = wall_toolpaths.getInnerContour();
    }
    else
    {
        inner_contour = outer_contour;
    }
    return inner_contour;
}

void Infill::generate(
    std::vector<VariableWidthLines>& toolpaths,
    Shape& result_polygons,
    OpenLinesSet& result_lines,
    const Settings& settings,
    int layer_idx,
    SectionType section_type,
    const std::shared_ptr<SierpinskiFillProvider>& cross_fill_provider,
    const std::shared_ptr<LightningLayer>& lightning_trees,
    const SliceMeshStorage* mesh,
    const Shape& prevent_small_exposed_to_air)
{
    if (outer_contour_.empty())
    {
        return;
    }

    inner_contour_ = generateWallToolPaths(toolpaths, outer_contour_, wall_line_count_, infill_line_width_, settings, layer_idx, section_type);
    scripta::log("infill_inner_contour_0", inner_contour_, section_type, layer_idx);

    inner_contour_ = inner_contour_.offset(infill_overlap_);

    // It does not make sense to print a pattern in a small region. So the infill region
    // is split into a small region that will be filled with walls and the normal region
    // that will be filled with the pattern. This split of regions is not needed if the
    // infill pattern is concentric or if the small_area_width is zero.
    if (pattern_ != EFillMethod::CONCENTRIC && small_area_width_ > 0)
    {
        const auto too_small_length = INT2MM(static_cast<double>(infill_line_width_) / 2.0);

        // Split the infill region in a narrow region and the normal region.
        Shape small_infill = inner_contour_;
        inner_contour_ = inner_contour_.offset(-small_area_width_ / 2);
        inner_contour_.removeSmallAreas(too_small_length * too_small_length, true);
        inner_contour_ = inner_contour_.offset(small_area_width_ / 2);
        inner_contour_ = inner_contour_.unionPolygons(prevent_small_exposed_to_air).intersection(small_infill);
        inner_contour_ = Simplify(max_resolution_, max_deviation_, 0).polygon(inner_contour_);
        small_infill = small_infill.difference(inner_contour_);

        // Small corners of a bigger area should not be considered narrow and are therefore added to the bigger area again.
        auto small_infill_parts = small_infill.splitIntoParts();
        small_infill.clear();
        for (const auto& small_infill_part : small_infill_parts)
        {
            if (small_infill_part.offset(-infill_line_width_ / 2).offset(infill_line_width_ / 2).area() < infill_line_width_ * infill_line_width_ * 10
                && ! inner_contour_.intersection(small_infill_part.offset(infill_line_width_ / 4)).empty())
            {
                inner_contour_.push_back(small_infill_part);
            }
            else
            {
                // the part must still be printed, so re-add it
                small_infill.push_back(small_infill_part);
            }
        }
        inner_contour_ = inner_contour_.unionPolygons();

        // Fill narrow area with walls.
        const size_t narrow_wall_count = small_area_width_ / infill_line_width_ + 1;
        WallToolPaths wall_toolpaths(small_infill, infill_line_width_, narrow_wall_count, 0, settings, layer_idx, section_type);
        std::vector<VariableWidthLines> small_infill_paths = wall_toolpaths.getToolPaths();
        scripta::log(
            "infill_small_infill_paths_0",
            small_infill_paths,
            section_type,
            layer_idx,
            scripta::CellVDI{ "is_closed", &ExtrusionLine::is_closed_ },
            scripta::CellVDI{ "is_odd", &ExtrusionLine::is_odd_ },
            scripta::CellVDI{ "inset_idx", &ExtrusionLine::inset_idx_ },
            scripta::PointVDI{ "width", &ExtrusionJunction::w_ },
            scripta::PointVDI{ "perimeter_index", &ExtrusionJunction::perimeter_index_ });
        for (const auto& small_infill_path : small_infill_paths)
        {
            toolpaths.emplace_back(small_infill_path);
        }
    }
    scripta::log("infill_inner_contour_1", inner_contour_, section_type, layer_idx);

    // apply an extra offset in case the pattern prints along the sides of the area.
    if (pattern_ == EFillMethod::ZIG_ZAG // Zig-zag prints the zags along the walls.
        || (zig_zaggify_
            && (pattern_ == EFillMethod::LINES // Zig-zaggified infill patterns print their zags along the walls.
                || pattern_ == EFillMethod::TRIANGLES || pattern_ == EFillMethod::GRID || pattern_ == EFillMethod::CUBIC || pattern_ == EFillMethod::TETRAHEDRAL
                || pattern_ == EFillMethod::QUARTER_CUBIC || pattern_ == EFillMethod::TRIHEXAGON || pattern_ == EFillMethod::GYROID || pattern_ == EFillMethod::CROSS
                || pattern_ == EFillMethod::CROSS_3D))
        || infill_multiplier_ % 2
               == 0) // Multiplied infill prints loops of infill, partly along the walls, if even. For odd multipliers >1 it gets offset by the multiply algorithm itself.
    {
        inner_contour_ = inner_contour_.offset(-infill_line_width_ / 2);
        inner_contour_ = Simplify(max_resolution_, max_deviation_, 0).polygon(inner_contour_);
    }
    scripta::log("infill_inner_contour_2", inner_contour_, section_type, layer_idx);

    if (infill_multiplier_ > 1)
    {
        bool zig_zaggify_real = zig_zaggify_;
        if (infill_multiplier_ % 2 == 0)
        {
            zig_zaggify_ = false;
        }
        Shape generated_result_polygons;
        OpenLinesSet generated_result_lines;

        _generate(toolpaths, generated_result_polygons, generated_result_lines, settings, cross_fill_provider, lightning_trees, mesh);

        zig_zaggify_ = zig_zaggify_real;
        multiplyInfill(generated_result_polygons, generated_result_lines);
        result_polygons.push_back(generated_result_polygons);
        result_lines.push_back(generated_result_lines);
    }
    else
    {
        //_generate may clear() the generated_result_lines, but this is an output variable that may contain data before we start.
        // So make sure we provide it with a Shape that is safe to clear and only add stuff to result_lines.
        Shape generated_result_polygons;
        OpenLinesSet generated_result_lines;

        _generate(toolpaths, generated_result_polygons, generated_result_lines, settings, cross_fill_provider, lightning_trees, mesh);

        result_polygons.push_back(generated_result_polygons);
        result_lines.push_back(generated_result_lines);
    }
    scripta::log("infill_result_polygons_0", result_polygons, section_type, layer_idx);
    scripta::log("infill_result_lines_0", result_lines, section_type, layer_idx);
    scripta::log(
        "infill_toolpaths_0",
        toolpaths,
        section_type,
        layer_idx,
        scripta::CellVDI{ "is_closed", &ExtrusionLine::is_closed_ },
        scripta::CellVDI{ "is_odd", &ExtrusionLine::is_odd_ },
        scripta::CellVDI{ "inset_idx", &ExtrusionLine::inset_idx_ },
        scripta::PointVDI{ "width", &ExtrusionJunction::w_ },
        scripta::PointVDI{ "perimeter_index", &ExtrusionJunction::perimeter_index_ });
    if (connect_polygons_)
    {
        // remove too small polygons
        coord_t snap_distance = infill_line_width_ * 2; // polygons with a span of max 1 * nozzle_size are too small
        auto it = std::remove_if(
            result_polygons.begin(),
            result_polygons.end(),
            [snap_distance](const Polygon& poly)
            {
                return poly.shorterThan(snap_distance);
            });
        result_polygons.erase(it, result_polygons.end());

        PolygonConnector connector(infill_line_width_);
        connector.add(result_polygons);
        connector.add(toolpaths);
        Shape connected_polygons;
        std::vector<VariableWidthLines> connected_paths;
        connector.connect(connected_polygons, connected_paths);
        result_polygons = connected_polygons;
        toolpaths = connected_paths;
        scripta::log("infill_result_polygons_1", result_polygons, section_type, layer_idx);
        scripta::log("infill_result_lines_1", result_lines, section_type, layer_idx);
        scripta::log(
            "infill_toolpaths_1",
            toolpaths,
            section_type,
            layer_idx,
            scripta::CellVDI{ "is_closed", &ExtrusionLine::is_closed_ },
            scripta::CellVDI{ "is_odd", &ExtrusionLine::is_odd_ },
            scripta::CellVDI{ "inset_idx", &ExtrusionLine::inset_idx_ },
            scripta::PointVDI{ "width", &ExtrusionJunction::w_ },
            scripta::PointVDI{ "perimeter_index", &ExtrusionJunction::perimeter_index_ });
    }
}

void Infill::_generate(
    std::vector<VariableWidthLines>& toolpaths,
    Shape& result_polygons,
    OpenLinesSet& result_lines,
    const Settings& settings,
    const std::shared_ptr<SierpinskiFillProvider>& cross_fill_provider,
    const std::shared_ptr<LightningLayer>& lightning_trees,
    const SliceMeshStorage* mesh)
{
    if (inner_contour_.empty())
        return;
    if (line_distance_ == 0)
        return;

    switch (pattern_)
    {
    case EFillMethod::GRID:
        generateGridInfill(result_lines);
        break;
    case EFillMethod::LINES:
        generateLineInfill(result_lines, line_distance_, fill_angle_, 0);
        break;
    case EFillMethod::CUBIC:
        generateCubicInfill(result_lines);
        break;
    case EFillMethod::TETRAHEDRAL:
        generateTetrahedralInfill(result_lines);
        break;
    case EFillMethod::QUARTER_CUBIC:
        generateQuarterCubicInfill(result_lines);
        break;
    case EFillMethod::TRIANGLES:
        generateTriangleInfill(result_lines);
        break;
    case EFillMethod::TRIHEXAGON:
        generateTrihexagonInfill(result_lines);
        break;
    case EFillMethod::CONCENTRIC:
        generateConcentricInfill(toolpaths, settings);
        break;
    case EFillMethod::ZIG_ZAG:
        generateZigZagInfill(result_lines, line_distance_, fill_angle_);
        break;
    case EFillMethod::CUBICSUBDIV:
        if (! mesh)
        {
            spdlog::error("Cannot generate Cubic Subdivision infill without a mesh!");
            break;
        }
        generateCubicSubDivInfill(result_lines, *mesh);
        break;
    case EFillMethod::CROSS:
    case EFillMethod::CROSS_3D:
        if (! cross_fill_provider)
        {
            spdlog::error("Cannot generate Cross infill without a cross fill provider!\n");
            break;
        }
        generateCrossInfill(*cross_fill_provider, result_polygons, result_lines);
        break;
    case EFillMethod::GYROID:
        generateGyroidInfill(result_lines, result_polygons);
        break;
    case EFillMethod::LIGHTNING:
        assert(lightning_trees); // "Cannot generate Lightning infill without a generator!\n"
        generateLightningInfill(lightning_trees, result_lines);
        break;
    case EFillMethod::PLUGIN:
    {
#ifdef ENABLE_PLUGINS // FIXME: I don't like this conditional block outside of the plugin scope.
        auto [toolpaths_, generated_result_polygons_, generated_result_lines_] = slots::instance().generate<plugins::v0::SlotID::INFILL_GENERATE>(
            inner_contour_,
            mesh ? mesh->settings.get<std::string>("infill_pattern") : settings.get<std::string>("infill_pattern"),
            mesh ? mesh->settings : settings,
            z_);
        toolpaths.insert(toolpaths.end(), toolpaths_.begin(), toolpaths_.end());
        result_polygons.push_back(generated_result_polygons_);
        result_lines.push_back(generated_result_lines_);
#endif
        break;
    }
    default:
        spdlog::error("Fill pattern has unknown value.\n");
        break;
    }

    if (connect_lines_)
    {
        // The list should be empty because it will be again filled completely. Otherwise, might have double lines.
        assert(result_lines.empty());
        result_lines.clear();
        connectLines(result_lines);
    }

    Simplify simplifier(max_resolution_, max_deviation_, 0);
    result_polygons = simplifier.polygon(result_polygons);

    if (! skip_line_stitching_
        && (zig_zaggify_ || pattern_ == EFillMethod::CROSS || pattern_ == EFillMethod::CROSS_3D || pattern_ == EFillMethod::CUBICSUBDIV || pattern_ == EFillMethod::GYROID
            || pattern_ == EFillMethod::ZIG_ZAG))
    { // don't stich for non-zig-zagged line infill types
        OpenLinesSet stitched_lines;
        OpenPolylineStitcher::stitch(result_lines, stitched_lines, result_polygons, infill_line_width_);
        result_lines = std::move(stitched_lines);
    }
    result_lines = simplifier.polyline(result_lines);
}

void Infill::multiplyInfill(Shape& result_polygons, OpenLinesSet& result_lines)
{
    if (pattern_ == EFillMethod::CONCENTRIC)
    {
        result_polygons = result_polygons.processEvenOdd(); // make into areas
    }

    bool odd_multiplier = infill_multiplier_ % 2 == 1;
    coord_t offset = (odd_multiplier) ? infill_line_width_ : infill_line_width_ / 2;

    // Get the first offset these are mirrored from the original center line
    Shape result;
    Shape first_offset;
    {
        const Shape first_offset_lines = result_lines.offset(offset); // make lines on both sides of the input lines
        const Shape first_offset_polygons_inward = result_polygons.offset(-offset); // make lines on the inside of the input polygons
        const Shape first_offset_polygons_outward = result_polygons.offset(offset); // make lines on the other side of the input polygons
        const Shape first_offset_polygons = first_offset_polygons_outward.difference(first_offset_polygons_inward);
        first_offset = first_offset_lines.unionPolygons(
            first_offset_polygons); // usually we only have either lines or polygons, but this code also handles an infill pattern which generates both
        if (zig_zaggify_)
        {
            first_offset = inner_contour_.difference(first_offset);
        }
    }
    result.push_back(first_offset);

    // Create the additional offsets from the first offsets, generated earlier, the direction of these offsets is
    // depended on whether these lines should be connected or not.
    if (infill_multiplier_ > 3)
    {
        Shape reference_polygons = first_offset;
        const size_t multiplier = infill_multiplier_ / 2;

        const int extra_offset = mirror_offset_ ? -infill_line_width_ : infill_line_width_;
        for (size_t infill_line = 1; infill_line < multiplier; ++infill_line)
        {
            Shape extra_polys = reference_polygons.offset(extra_offset);
            result.push_back(extra_polys);
            reference_polygons = std::move(extra_polys);
        }
    }
    if (zig_zaggify_)
    {
        result = result.intersection(inner_contour_);
    }

    // Remove the original center lines when there are an even number of lines required.
    if (! odd_multiplier)
    {
        result_polygons.clear();
        result_lines.clear();
    }
    result_polygons.push_back(result);
    if (! zig_zaggify_)
    {
        OpenLinesSet polylines = inner_contour_.intersection(static_cast<LinesSet<Polygon>>(result_polygons));
        result_polygons.clear();
        OpenPolylineStitcher::stitch(polylines, result_lines, result_polygons, infill_line_width_);
    }
}

void Infill::generateGyroidInfill(OpenLinesSet& result_lines, Shape& result_polygons)
{
    OpenLinesSet line_segments;
    GyroidInfill::generateTotalGyroidInfill(line_segments, zig_zaggify_, line_distance_, inner_contour_, z_);
    OpenPolylineStitcher::stitch(line_segments, result_lines, result_polygons, infill_line_width_);
}

void Infill::generateLightningInfill(const std::shared_ptr<LightningLayer>& trees, OpenLinesSet& result_lines)
{
    // Don't need to support areas smaller than line width, as they are always within radius:
    if (std::abs(inner_contour_.area()) < infill_line_width_ || ! trees)
    {
        return;
    }
    result_lines.push_back(trees->convertToLines(inner_contour_, infill_line_width_));
}

void Infill::generateConcentricInfill(std::vector<VariableWidthLines>& toolpaths, const Settings& settings)
{
    const coord_t min_area = infill_line_width_ * infill_line_width_;

    Shape current_inset = inner_contour_;
    Simplify simplifier(settings);
    while (true)
    {
        // If line_distance is 0, start from the same contour as the previous line, except where the previous line closed up the shape.
        // So we add the whole nominal line width first (to allow lines to be closer together than 1 line width if the line distance is smaller) and then subtract line_distance.
        current_inset = current_inset.offset(infill_line_width_ - line_distance_);
        current_inset = simplifier.polygon(current_inset); // Many insets lead to increasingly detailed shapes. Simplify to speed up processing.
        if (current_inset.area() < min_area) // So small that it's inconsequential. Stop here.
        {
            break;
        }

        constexpr size_t inset_wall_count = 1; // 1 wall at a time.
        constexpr coord_t wall_0_inset = 0; // Don't apply any outer wall inset for these. That's just for the outer wall.
        WallToolPaths wall_toolpaths(current_inset, infill_line_width_, inset_wall_count, wall_0_inset, settings, 0, SectionType::CONCENTRIC_INFILL); // FIXME: @jellespijker pass
                                                                                                                                                      // the correct layer
        const std::vector<VariableWidthLines> inset_paths = wall_toolpaths.getToolPaths();
        toolpaths.insert(toolpaths.end(), inset_paths.begin(), inset_paths.end());

        current_inset = wall_toolpaths.getInnerContour();
    }
}

void Infill::generateGridInfill(OpenLinesSet& result)
{
    generateLineInfill(result, line_distance_, fill_angle_, 0);
    generateLineInfill(result, line_distance_, fill_angle_ + 90, 0);
}

void Infill::generateCubicInfill(OpenLinesSet& result)
{
    const coord_t shift = one_over_sqrt_2 * z_;
    generateLineInfill(result, line_distance_, fill_angle_, shift);
    generateLineInfill(result, line_distance_, fill_angle_ + 120, shift);
    generateLineInfill(result, line_distance_, fill_angle_ + 240, shift);
}

void Infill::generateTetrahedralInfill(OpenLinesSet& result)
{
    generateHalfTetrahedralInfill(0.0, 0, result);
    generateHalfTetrahedralInfill(0.0, 90, result);
}

void Infill::generateQuarterCubicInfill(OpenLinesSet& result)
{
    generateHalfTetrahedralInfill(0.0, 0, result);
    generateHalfTetrahedralInfill(0.5, 90, result);
}

void Infill::generateHalfTetrahedralInfill(double pattern_z_shift, int angle_shift, OpenLinesSet& result)
{
    const coord_t period = line_distance_ * 2;
    coord_t shift = coord_t(one_over_sqrt_2 * (z_ + pattern_z_shift * period * 2)) % period;
    shift = std::min(shift, period - shift); // symmetry due to the fact that we are applying the shift in both directions
    shift = std::min(shift, period / 2 - infill_line_width_ / 2); // don't put lines too close to each other
    shift = std::max(shift, infill_line_width_ / 2); // don't put lines too close to each other
    generateLineInfill(result, period, fill_angle_ + angle_shift, shift);
    generateLineInfill(result, period, fill_angle_ + angle_shift, -shift);
}

void Infill::generateTriangleInfill(OpenLinesSet& result)
{
    generateLineInfill(result, line_distance_, fill_angle_, 0);
    generateLineInfill(result, line_distance_, fill_angle_ + 60, 0);
    generateLineInfill(result, line_distance_, fill_angle_ + 120, 0);
}

void Infill::generateTrihexagonInfill(OpenLinesSet& result)
{
    generateLineInfill(result, line_distance_, fill_angle_, 0);
    generateLineInfill(result, line_distance_, fill_angle_ + 60, 0);
    generateLineInfill(result, line_distance_, fill_angle_ + 120, line_distance_ / 2);
}

void Infill::generateCubicSubDivInfill(OpenLinesSet& result, const SliceMeshStorage& mesh)
{
    OpenLinesSet uncropped;
    mesh.base_subdiv_cube->generateSubdivisionLines(z_, uncropped);
    constexpr bool restitch = false; // cubic subdivision lines are always single line segments - not polylines consisting of multiple segments.
    result = outer_contour_.offset(infill_overlap_).intersection(uncropped, restitch);
}

void Infill::generateCrossInfill(const SierpinskiFillProvider& cross_fill_provider, Shape& result_polygons, OpenLinesSet& result_lines)
{
    Polygon cross_pattern_polygon = cross_fill_provider.generate(pattern_, z_, infill_line_width_, pocket_size_);

    if (cross_pattern_polygon.empty())
    {
        return;
    }

    if (zig_zaggify_)
    {
        Shape cross_pattern_polygons;
        cross_pattern_polygons.push_back(cross_pattern_polygon);
        result_polygons.push_back(inner_contour_.intersection(cross_pattern_polygons));
    }
    else
    {
        // make the polyline closed in order to handle cross_pattern_polygon as a polyline, rather than a closed polygon
        OpenLinesSet cross_pattern_polylines;
        cross_pattern_polylines.push_back(cross_pattern_polygon.toPseudoOpenPolyline());
        OpenLinesSet poly_lines = inner_contour_.intersection(cross_pattern_polylines);
        OpenPolylineStitcher::stitch(poly_lines, result_lines, result_polygons, infill_line_width_);
    }
}

void Infill::addLineInfill(
    OpenLinesSet& result,
    const PointMatrix& rotation_matrix,
    const int scanline_min_idx,
    const int line_distance,
    const AABB boundary,
    std::vector<std::vector<coord_t>>& cut_list,
    coord_t shift)
{
    assert(! connect_lines_ && "connectLines() should add the infill lines, not addLineInfill");

    unsigned int scanline_idx = 0;
    for (coord_t x = scanline_min_idx * line_distance + shift; x < boundary.max_.X; x += line_distance)
    {
        if (scanline_idx >= cut_list.size())
        {
            break;
        }
        std::vector<coord_t>& crossings = cut_list[scanline_idx];
        std::sort(crossings.begin(), crossings.end()); // sort by increasing Y coordinates
        for (unsigned int crossing_idx = 0; crossing_idx + 1 < crossings.size(); crossing_idx += 2)
        {
            if (crossings[crossing_idx + 1] - crossings[crossing_idx] < infill_line_width_ / 5)
            { // segment is too short to create infill
                continue;
            }
            result.addSegment(rotation_matrix.unapply(Point2LL(x, crossings[crossing_idx])), rotation_matrix.unapply(Point2LL(x, crossings[crossing_idx + 1])));
        }
        scanline_idx += 1;
    }
}

coord_t Infill::getShiftOffsetFromInfillOriginAndRotation(const double& infill_rotation)
{
    if (infill_origin_.X != 0 || infill_origin_.Y != 0)
    {
        const double rotation_rads = infill_rotation * std::numbers::pi / 180;
        return infill_origin_.X * std::cos(rotation_rads) - infill_origin_.Y * std::sin(rotation_rads);
    }
    return 0;
}

void Infill::generateLineInfill(OpenLinesSet& result, int line_distance, const double& infill_rotation, coord_t shift)
{
    shift += getShiftOffsetFromInfillOriginAndRotation(infill_rotation);
    PointMatrix rotation_matrix(infill_rotation);
    NoZigZagConnectorProcessor lines_processor(rotation_matrix, result);
    bool connected_zigzags = false;
    generateLinearBasedInfill(result, line_distance, rotation_matrix, lines_processor, connected_zigzags, shift);
}


void Infill::generateZigZagInfill(OpenLinesSet& result, const coord_t line_distance, const double& infill_rotation)
{
    const coord_t shift = getShiftOffsetFromInfillOriginAndRotation(infill_rotation);

    PointMatrix rotation_matrix(infill_rotation);
    ZigzagConnectorProcessor zigzag_processor(rotation_matrix, result, use_endpieces_, connected_zigzags_, skip_some_zags_, zag_skip_count_);
    generateLinearBasedInfill(result, line_distance, rotation_matrix, zigzag_processor, connected_zigzags_, shift);
}

/*
 * algorithm:
 * 1. for each line segment of each polygon:
 *      store the intersections of that line segment with all scanlines in a mapping (vector of vectors) from scanline to intersections
 *      (zigzag): add boundary segments to result
 * 2. for each scanline:
 *      sort the associated intersections
 *      and connect them using the even-odd rule
 *
 * rough explanation of the zigzag algorithm:
 * while walking around (each) polygon (1.)
 *  if polygon intersects with even scanline
 *      start boundary segment (add each following segment to the [result])
 *  when polygon intersects with a scanline again
 *      stop boundary segment (stop adding segments to the [result])
 *  (see infill/ZigzagConnectorProcessor.h for actual implementation details)
 *
 *
 * we call the areas between two consecutive scanlines a 'scansegment'.
 * Scansegment x is the area between scanline x and scanline x+1
 * Edit: the term scansegment is wrong, since I call a boundary segment leaving from an even scanline to the left as belonging to an even scansegment,
 *  while I also call a boundary segment leaving from an even scanline toward the right as belonging to an even scansegment.
 */
void Infill::generateLinearBasedInfill(
    OpenLinesSet& result,
    const int line_distance,
    const PointMatrix& rotation_matrix,
    ZigzagConnectorProcessor& zigzag_connector_processor,
    const bool connected_zigzags,
    coord_t extra_shift)
{
    if (line_distance == 0 || inner_contour_.empty()) // No infill to generate (0% density) or no area to generate it in.
    {
        return;
    }

    Shape outline = inner_contour_; // Make a copy. We'll be rotating this outline to make intersections always horizontal, for better performance.
    outline.applyMatrix(rotation_matrix);

    coord_t shift = extra_shift + this->shift_;
    if (shift < 0)
    {
        shift = line_distance - (-shift) % line_distance;
    }
    else
    {
        shift = shift % line_distance;
    }

    AABB boundary(outline);

    int scanline_min_idx = computeScanSegmentIdx(boundary.min_.X - shift, line_distance);
    int line_count = computeScanSegmentIdx(boundary.max_.X - shift, line_distance) + 1 - scanline_min_idx;

    std::vector<std::vector<coord_t>> cut_list(line_count); // mapping from scanline to all intersections with polygon segments

    // When we find crossings, keep track of which crossing belongs to which scanline and to which polygon line segment.
    // Then we can later join two crossings together to form lines and still know what polygon line segments that infill line connected to.
    struct Crossing
    {
        Point2LL coordinate_;
        size_t polygon_index_;
        size_t vertex_index_;

        Crossing(Point2LL coordinate, size_t polygon_index, size_t vertex_index)
            : coordinate_(coordinate)
            , polygon_index_(polygon_index)
            , vertex_index_(vertex_index)
        {
        }

        bool operator<(const Crossing& other) const // Crossings will be ordered by their Y coordinate so that they get ordered along the scanline.
        {
            return coordinate_.Y < other.coordinate_.Y;
        }
    };
    std::vector<std::vector<Crossing>> crossings_per_scanline; // For each scanline, a list of crossings.
    const int min_scanline_index = computeScanSegmentIdx(boundary.min_.X - shift, line_distance) + 1;
    const int max_scanline_index = computeScanSegmentIdx(boundary.max_.X - shift, line_distance) + 1;
    crossings_per_scanline.resize(max_scanline_index - min_scanline_index);
    if (connect_lines_)
    {
        crossings_on_line_.resize(outline.size()); // One for each polygon.
    }

    for (size_t poly_idx = 0; poly_idx < outline.size(); poly_idx++)
    {
        const Polygon& poly = outline[poly_idx];
        if (connect_lines_)
        {
            crossings_on_line_[poly_idx].resize(poly.size()); // One for each line in this polygon.
        }
        Point2LL p0 = poly.back();
        zigzag_connector_processor.registerVertex(p0); // always adds the first point to ZigzagConnectorProcessorEndPieces::first_zigzag_connector when using a zigzag infill type

        for (size_t point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            Point2LL p1 = poly[point_idx];
            if (p1.X == p0.X)
            {
                zigzag_connector_processor.registerVertex(p1);
                // TODO: how to make sure it always adds the shortest line? (in order to prevent overlap with the zigzag connectors)
                // note: this is already a problem for normal infill, but hasn't really bothered anyone so far.
                p0 = p1;
                continue;
            }

            int scanline_idx0;
            int scanline_idx1;
            // this way of handling the indices takes care of the case where a boundary line segment ends exactly on a scanline:
            // in case the next segment moves back from that scanline either 2 or 0 scanline-boundary intersections are created
            // otherwise only 1 will be created, counting as an actual intersection
            int direction = 1;
            if (p0.X < p1.X)
            {
                scanline_idx0 = computeScanSegmentIdx(p0.X - shift, line_distance) + 1; // + 1 cause we don't cross the scanline of the first scan segment
                scanline_idx1
                    = computeScanSegmentIdx(p1.X - shift, line_distance); // -1 cause the vertex point is handled in the next segment (or not in the case which looks like >)
            }
            else
            {
                direction = -1;
                scanline_idx0
                    = computeScanSegmentIdx(p0.X - shift, line_distance); // -1 cause the vertex point is handled in the previous segment (or not in the case which looks like >)
                scanline_idx1 = computeScanSegmentIdx(p1.X - shift, line_distance) + 1; // + 1 cause we don't cross the scanline of the first scan segment
            }

            for (int scanline_idx = scanline_idx0; scanline_idx != scanline_idx1 + direction; scanline_idx += direction)
            {
                const int x = scanline_idx * line_distance + shift;
                const int y = p1.Y + (p0.Y - p1.Y) * (x - p1.X) / (p0.X - p1.X);
                assert(scanline_idx - scanline_min_idx >= 0 && scanline_idx - scanline_min_idx < int(cut_list.size()) && "reading infill cutlist index out of bounds!");
                cut_list[scanline_idx - scanline_min_idx].push_back(y);
                Point2LL scanline_linesegment_intersection(x, y);
                zigzag_connector_processor.registerScanlineSegmentIntersection(scanline_linesegment_intersection, scanline_idx, line_distance / 4);
                crossings_per_scanline[scanline_idx - min_scanline_index].emplace_back(scanline_linesegment_intersection, poly_idx, point_idx);
            }
            zigzag_connector_processor.registerVertex(p1);
            p0 = p1;
        }
        zigzag_connector_processor.registerPolyFinished();
    }

    if (connect_lines_)
    {
        // Gather all crossings per scanline and find out which crossings belong together, then store them in crossings_on_line.
        for (int scanline_index = min_scanline_index; scanline_index < max_scanline_index; scanline_index++)
        {
            auto& crossings = crossings_per_scanline[scanline_index - min_scanline_index];
            // Sorts them by Y coordinate.
            std::sort(crossings.begin(), crossings.end());
            // Combine each 2 subsequent crossings together.
            for (long crossing_index = 0; crossing_index < static_cast<long>(crossings.size()) - 1; crossing_index += 2)
            {
                const Crossing& first = crossings[crossing_index];
                const Crossing& second = crossings[crossing_index + 1];
                // Avoid creating zero length crossing lines
                const Point2LL unrotated_first = rotation_matrix.unapply(first.coordinate_);
                const Point2LL unrotated_second = rotation_matrix.unapply(second.coordinate_);
                if (unrotated_first == unrotated_second)
                {
                    continue;
                }
                InfillLineSegment* new_segment
                    = new InfillLineSegment(unrotated_first, first.vertex_index_, first.polygon_index_, unrotated_second, second.vertex_index_, second.polygon_index_);
                // Put the same line segment in the data structure twice: Once for each of the polygon line segment that it crosses.
                crossings_on_line_[first.polygon_index_][first.vertex_index_].push_back(new_segment);
                crossings_on_line_[second.polygon_index_][second.vertex_index_].push_back(new_segment);
            }
        }
    }
    else
    {
        if (cut_list.size() == 0)
        {
            return;
        }
        if (connected_zigzags && cut_list.size() == 1 && cut_list[0].size() <= 2)
        {
            return; // don't add connection if boundary already contains whole outline!
        }

        // We have to create our own lines when they are not created by the method connectLines.
        addLineInfill(result, rotation_matrix, scanline_min_idx, line_distance, boundary, cut_list, shift);
    }
}

void Infill::resolveIntersection(const coord_t at_distance, const Point2LL& intersect, Point2LL& connect_start, Point2LL& connect_end, InfillLineSegment* a, InfillLineSegment* b)
{
    // Select wich ends of the line need to 'bend'.
    const bool forward_line_a = a->end_ == connect_start;
    const bool forward_line_b = b->start_ == connect_end;
    auto& bend_a = forward_line_a ? a->end_bend_ : a->start_bend_;
    auto& bend_b = forward_line_b ? b->start_bend_ : b->end_bend_;
    auto& end_a = forward_line_a ? a->altered_end_ : a->altered_start_;
    auto& end_b = forward_line_b ? b->altered_start_ : b->altered_end_;

    // Set values ('pre existing' values are needed when feeding these as reference parameters to functions that need a value).
    assert(! bend_a.has_value());
    assert(! bend_b.has_value());
    bend_a.emplace(0, 0);
    bend_b.emplace(0, 0);

    // Find a bisector of the intersection; specifically, the one that crosses the connection & offset it by 1/2 distance to each side.
    constexpr auto large_enough_vec_len = 0xFFFFFFFF;
    const auto bisect = LinearAlg2D::getBisectorVector(intersect, connect_start, connect_end, large_enough_vec_len);
    const auto offset = ((at_distance / 2) * Point2LL(-bisect.Y, bisect.X)) / large_enough_vec_len;
    const auto q = intersect + offset;
    const auto r = q + bisect;
    const auto s = intersect - offset;
    const auto t = s + bisect;

    // In certain rare conditions, the lines do not actually intersect in a way that we can solve with the current algorithm.
    bool is_resolved = true;

    // Use both of the resulting lines to place the 'bends' by intersecting with the original line-segments.
    is_resolved &= LinearAlg2D::lineLineIntersection(q, r, a->start_, a->end_, bend_a.value()) && LinearAlg2D::pointIsProjectedBeyondLine(bend_a.value(), a->start_, a->end_) == 0;
    is_resolved &= LinearAlg2D::lineLineIntersection(s, t, b->start_, b->end_, bend_b.value()) && LinearAlg2D::pointIsProjectedBeyondLine(bend_b.value(), b->start_, b->end_) == 0;

    // Also set the new end-points
    is_resolved &= LinearAlg2D::lineLineIntersection(connect_start, connect_end, q, r, end_a) && LinearAlg2D::pointIsProjectedBeyondLine(end_a, connect_start, connect_end) == 0;
    is_resolved &= LinearAlg2D::lineLineIntersection(connect_start, connect_end, s, t, end_b) && LinearAlg2D::pointIsProjectedBeyondLine(end_b, connect_start, connect_end) == 0;

    if (is_resolved)
    {
        // The connecting line will be made from the end-points.
        connect_start = end_a;
        connect_end = end_b;
    }
    else
    {
        // Put everything that has now potentially become messed up, back.
        bend_a.reset();
        bend_b.reset();
    }
}

void Infill::connectLines(OpenLinesSet& result_lines)
{
    UnionFind<InfillLineSegment*> connected_lines; // Keeps track of which lines are connected to which.
    for (const std::vector<std::vector<InfillLineSegment*>>& crossings_on_polygon : crossings_on_line_)
    {
        for (const std::vector<InfillLineSegment*>& crossings_on_polygon_segment : crossings_on_polygon)
        {
            for (InfillLineSegment* infill_line : crossings_on_polygon_segment)
            {
                if (connected_lines.find(infill_line) == (size_t)-1)
                {
                    connected_lines.add(infill_line); // Put every line in there as a separate set.
                }
            }
        }
    }

    const auto half_line_distance_squared = (line_distance_ * line_distance_) / 4;
    for (size_t polygon_index = 0; polygon_index < inner_contour_.size(); polygon_index++)
    {
        const Polygon& inner_contour_polygon = inner_contour_[polygon_index];
        if (inner_contour_polygon.empty())
        {
            continue;
        }
        assert(crossings_on_line_.size() > polygon_index && "crossings dimension should be bigger then polygon index");
        std::vector<std::vector<InfillLineSegment*>>& crossings_on_polygon = crossings_on_line_[polygon_index];
        InfillLineSegment* previous_crossing = nullptr; // The crossing that we should connect to. If nullptr, we have been skipping until we find the next crossing.
        InfillLineSegment* previous_segment = nullptr; // The last segment we were connecting while drawing a line along the border.
        Point2LL vertex_before = inner_contour_polygon.back();
        for (size_t vertex_index = 0; vertex_index < inner_contour_polygon.size(); vertex_index++)
        {
            assert(crossings_on_polygon.size() > vertex_index && "crossings on line for the current polygon should be bigger then vertex index");
            std::vector<InfillLineSegment*>& crossings_on_polygon_segment = crossings_on_polygon[vertex_index];
            Point2LL vertex_after = inner_contour_polygon[vertex_index];

            // Sort crossings on every line by how far they are from their initial point.
            std::sort(
                crossings_on_polygon_segment.begin(),
                crossings_on_polygon_segment.end(),
                [&vertex_before, polygon_index, vertex_index](InfillLineSegment* left_hand_side, InfillLineSegment* right_hand_side)
                {
                    // Find the two endpoints that are relevant.
                    const bool choose_left = (left_hand_side->start_segment_ == vertex_index && left_hand_side->start_polygon_ == polygon_index);
                    const bool choose_right = (right_hand_side->start_segment_ == vertex_index && right_hand_side->start_polygon_ == polygon_index);
                    const Point2LL left_hand_point = choose_left ? left_hand_side->start_ : left_hand_side->end_;
                    const Point2LL right_hand_point = choose_right ? right_hand_side->start_ : right_hand_side->end_;
                    return vSize(left_hand_point - vertex_before) < vSize(right_hand_point - vertex_before);
                });

            for (InfillLineSegment* crossing : crossings_on_polygon_segment)
            {
                if (! previous_crossing) // If we're not yet drawing, then we have been trying to find the next vertex. We found it! Let's start drawing.
                {
                    previous_crossing = crossing;
                    previous_segment = crossing;
                }
                else
                {
                    const size_t crossing_handle = connected_lines.find(crossing);
                    assert(crossing_handle != (size_t)-1);
                    const size_t previous_crossing_handle = connected_lines.find(previous_crossing);
                    assert(previous_crossing_handle != (size_t)-1);
                    if (crossing_handle == previous_crossing_handle)
                    {
                        // These two infill lines are already connected. Don't create a loop now. Continue connecting with the next crossing.
                        continue;
                    }

                    // Join two infill lines together with a connecting line.
                    // Here the InfillLineSegments function as a linked list, so that they can easily be joined.
                    const bool previous_forward = (previous_segment->start_segment_ == vertex_index && previous_segment->start_polygon_ == polygon_index);
                    const bool next_forward = (crossing->start_segment_ == vertex_index && crossing->start_polygon_ == polygon_index);
                    Point2LL& previous_point = previous_forward ? previous_segment->start_ : previous_segment->end_;
                    Point2LL& next_point = next_forward ? crossing->start_ : crossing->end_;

                    InfillLineSegment* new_segment;
                    // If the segment is near length, we avoid creating it but still want to connect the crossing with the previous segment.
                    if (previous_point == next_point)
                    {
                        (previous_forward ? previous_segment->previous_ : previous_segment->next_) = crossing;
                        new_segment = previous_segment;
                    }
                    else
                    {
                        constexpr coord_t epsilon_sqd = 25;

                        // Resolve any intersections of the fill lines close to the boundary, by inserting extra points so the lines don't create a tiny 'loop'.
                        Point2LL intersect;
                        if (vSize2(previous_point - next_point) < half_line_distance_squared
                            && LinearAlg2D::lineLineIntersection(previous_segment->start_, previous_segment->end_, crossing->start_, crossing->end_, intersect)
                            && LinearAlg2D::pointIsProjectedBeyondLine(intersect, previous_segment->start_, previous_segment->end_) == 0
                            && LinearAlg2D::pointIsProjectedBeyondLine(intersect, crossing->start_, crossing->end_) == 0 && vSize2(previous_point - intersect) > epsilon_sqd
                            && vSize2(next_point - intersect) > epsilon_sqd)
                        {
                            resolveIntersection(infill_line_width_, intersect, previous_point, next_point, previous_segment, crossing);
                        }

                        // A connecting line between them.
                        new_segment = new InfillLineSegment(previous_point, vertex_index, polygon_index, next_point, vertex_index, polygon_index);
                        new_segment->altered_start_ = previous_point;
                        new_segment->altered_end_ = next_point;
                        new_segment->previous_ = previous_segment;
                        (previous_forward ? previous_segment->previous_ : previous_segment->next_) = new_segment;
                        new_segment->next_ = crossing;
                    }

                    (next_forward ? crossing->previous_ : crossing->next_) = new_segment;
                    connected_lines.unite(crossing_handle, previous_crossing_handle);
                    previous_crossing = nullptr;
                    previous_segment = nullptr;
                }
            }

            // Upon going to the next vertex, if we're drawing, put an extra vertex in our infill lines.
            if (previous_crossing)
            {
                InfillLineSegment* new_segment;

                const bool choose_side = (vertex_index == previous_segment->start_segment_ && polygon_index == previous_segment->start_polygon_);
                const auto& previous_side = choose_side ? previous_segment->start_ : previous_segment->end_;
                if (previous_side == vertex_after)
                {
                    // Edge case when an infill line ends directly on top of vertex_after: We skip the extra connecting line segment, as that would be 0-length.
                    previous_segment = nullptr;
                    previous_crossing = nullptr;
                }
                else
                {
                    new_segment
                        = new InfillLineSegment(previous_side, vertex_index, polygon_index, vertex_after, (vertex_index + 1) % inner_contour_[polygon_index].size(), polygon_index);
                    (choose_side ? previous_segment->previous_ : previous_segment->next_) = new_segment;
                    new_segment->previous_ = previous_segment;
                    previous_segment = new_segment;
                }
            }

            vertex_before = vertex_after;
            crossings_on_polygon_segment.clear();
        }
    }

    // Save all lines, now connected, to the output.
    std::unordered_set<size_t> completed_groups;
    for (InfillLineSegment* infill_line : connected_lines)
    {
        const size_t group = connected_lines.find(infill_line);
        if (completed_groups.find(group) != completed_groups.end()) // We already completed this group.
        {
            continue;
        }

        // Find where the polyline ends by searching through previous and next lines.
        // Note that the "previous" and "next" lines don't necessarily match up though, because the direction while connecting infill lines was not yet known.
        Point2LL previous_vertex = infill_line->start_; // Take one side arbitrarily to start from. This variable indicates the vertex that connects to the previous line.
        InfillLineSegment* current_infill_line = infill_line;
        while (current_infill_line->next_ && current_infill_line->previous_) // Until we reached an endpoint.
        {
            const bool choose_side = (previous_vertex == current_infill_line->start_);
            const Point2LL next_vertex = choose_side ? current_infill_line->end_ : current_infill_line->start_;
            current_infill_line = choose_side ? current_infill_line->next_ : current_infill_line->previous_;
            previous_vertex = next_vertex;
        }

        // Now go along the linked list of infill lines and output the infill lines to the actual result.
        OpenPolyline& result_line = result_lines.newLine();
        InfillLineSegment* old_line = current_infill_line;
        if (current_infill_line->previous_)
        {
            current_infill_line->swapDirection();
        }
        current_infill_line->appendTo(result_line);
        previous_vertex = current_infill_line->end_;
        current_infill_line = current_infill_line->next_;
        delete old_line;
        while (current_infill_line)
        {
            old_line = current_infill_line; // We'll delete this after we've traversed to the next line.
            if (previous_vertex != current_infill_line->start_)
            {
                current_infill_line->swapDirection();
            }
            const Point2LL next_vertex = current_infill_line->end_; // Opposite side of the line.
            constexpr bool polyline_break = false;
            current_infill_line->appendTo(result_line, polyline_break);
            current_infill_line = current_infill_line->next_;
            previous_vertex = next_vertex;
            delete old_line;
        }

        completed_groups.insert(group);
    }
}

bool Infill::InfillLineSegment::operator==(const InfillLineSegment& other) const
{
    return start_ == other.start_ && end_ == other.end_;
}

void Infill::InfillLineSegment::swapDirection()
{
    std::swap(start_, end_);
    std::swap(altered_start_, altered_end_);
    std::swap(start_bend_, end_bend_);
    std::swap(next_, previous_);
}

void Infill::InfillLineSegment::appendTo(OpenPolyline& result_polyline, const bool include_start)
{
    if (include_start)
    {
        result_polyline.push_back(altered_start_);
    }
    if (start_bend_.has_value())
    {
        result_polyline.push_back(start_bend_.value());
    }
    if (end_bend_.has_value())
    {
        result_polyline.push_back(end_bend_.value());
    }
    result_polyline.push_back(altered_end_);
}

} // namespace cura
