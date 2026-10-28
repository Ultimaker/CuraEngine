// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <spdlog/spdlog.h>

#include "SliceDataStorage.h"
#include "TreeModelVolumes.h"
#include "TreeSupportSettings.h"
#include "geometry/LinesSet.h"
#include "geometry/OpenPolyline.h"
#include "geometry/Polygon.h"
#include "infill/Infill.h"
#include "polyclipping/clipper.hpp"
#include "types/EFillMethod.h"
#include "utils/Coord_t.h"

namespace curaengine
{

class TreeSupportUtils
{
public:
    /*!
     * \brief Adds the implicit line from the last vertex of a Polygon to the first one.
     *
     * \param poly[in] The Polygons object, of which its lines should be extended.
     * \return A Polygons object with explicit line from the last vertex of a Polygon to the first one added.
     */
    static geometry::OpenLinesSet toPolylines(const geometry::Shape& poly)
    {
        geometry::OpenLinesSet result;
        for (const auto& path : poly)
        {
            result.push_back(path.toPseudoOpenPolyline());
        }
        return result;
    }


    /*!
     * \brief Converts toolpaths to a Polygons object
     *
     * \param toolpaths[in] The toolpaths.
     * \return A Polygons object.
     */
    [[nodiscard]] static geometry::OpenLinesSet toPolylines(const std::vector<utils::VariableWidthLines> toolpaths)
    {
        geometry::OpenLinesSet result;
        for (const utils::VariableWidthLines& lines : toolpaths)
        {
            for (const utils::ExtrusionLine& line : lines)
            {
                if (line.empty())
                {
                    continue;
                }

                geometry::OpenPolyline result_line;
                for (const utils::ExtrusionJunction& junction : line)
                {
                    result_line.push_back(junction.p_);
                }

                if (line.is_closed_)
                {
                    result_line.push_back(line[0].p_);
                }

                result.push_back(result_line);
            }
        }
        return result;
    }


    /*!
     * \brief Returns Polylines representing the (infill) lines that will result in slicing the given area
     *
     * \param area[in] The area that has to be filled with infill.
     * \param config[in] The settings for generating the patterns.
     * \param roof[in] Whether the roofing or regular support settings should be used.
     * \param layer_idx[in] The current layer index.
     * \param support_infill_distance[in] The distance that should be between the infill lines.
     * \param cross_fill_provider[in] A SierpinskiFillProvider required for cross infill.
     * \param include_walls[in] If the result should also contain walls, or only the infill.
     * todo doku
     * \return A Polygons object that represents the resulting infill lines.
     */
    [[nodiscard]] static geometry::OpenLinesSet generateSupportInfillLines(
        const geometry::Shape& area,
        const TreeSupportSettings& config,
        bool roof,
        types::LayerIndex layer_idx,
        utils::coord_t support_infill_distance,
        std::shared_ptr<infill::SierpinskiFillProvider> cross_fill_provider,
        bool include_walls,
        bool generate_support_supporting = false)
    {
        geometry::Shape gaps;
        // As we effectivly use lines to place our supportPoints we may use the Infill class for it, while not made for it, it works perfectly.

        const types::EFillMethod pattern = generate_support_supporting ? types::EFillMethod::GRID : roof ? config.roof_pattern : config.support_pattern;

        if (pattern == types::EFillMethod::LIGHTNING)
        {
            // This can't be done for lightning infill since it requires the support to be generated first. Final result is good enough though,
            // because it propagates to properly support the top area, compensating for the same feature of the tree support.
            return {};
        }

        const bool zig_zaggify_infill = roof ? pattern == types::EFillMethod::ZIG_ZAG : config.zig_zaggify_support;
        const bool connect_polygons = false;
        constexpr utils::coord_t support_roof_overlap = 0;
        constexpr size_t infill_multiplier = 1;
        const int support_shift = roof ? 0 : support_infill_distance / 2;
        const utils::coord_t wall_line_count
            = include_walls ? (! roof ? config.support_wall_thickness : static_cast<utils::coord_t>(config.support_roof_wall_count) * config.support_roof_line_width) : utils::coord_t{ 0 };
        constexpr utils::coord_t narrow_area_width = 0;
        const geometry::Point2LL infill_origin;
        constexpr bool skip_stitching = false;
        constexpr bool fill_gaps = true;
        constexpr bool use_endpieces = true;
        const bool connected_zigzags = roof ? false : config.connect_zigzags;
        const bool skip_some_zags = roof ? false : config.skip_some_zags;
        const size_t zag_skip_count = roof ? 0 : config.zag_skip_count;
        constexpr utils::coord_t pocket_size = 0;
        std::vector<types::AngleDegrees> angles = roof ? config.support_roof_angles : config.support_infill_angles;
        std::vector<utils::VariableWidthLines> toolpaths;

        const utils::coord_t z = config.getActualZ(layer_idx);
        const size_t divisor = angles.size();
        const size_t index = ((layer_idx % divisor) + divisor) % divisor;
        const types::AngleDegrees fill_angle = angles[index];
        infill::Infill roof_computation(
            pattern,
            zig_zaggify_infill,
            connect_polygons,
            area,
            roof ? config.support_roof_line_width : config.support_line_width,
            support_infill_distance,
            support_roof_overlap,
            infill_multiplier,
            fill_angle,
            z,
            support_shift,
            config.maximum_resolution,
            config.maximum_deviation,
            wall_line_count,
            narrow_area_width,
            infill_origin,
            skip_stitching,
            fill_gaps,
            connected_zigzags,
            use_endpieces,
            skip_some_zags,
            zag_skip_count,
            pocket_size);

        geometry::Shape areas;
        geometry::OpenLinesSet lines;
        roof_computation.generate(toolpaths, areas, lines, config.settings, layer_idx, utils::SectionType::SUPPORT, cross_fill_provider);
        lines.push_back(toPolylines(areas));
        lines.push_back(toPolylines(toolpaths));
        return lines;
    }

    /*!
     * \brief Unions two Shape. Ensures that if the input is non empty that the output also will be non empty.
     * \param first[in] The first Polygon.
     * \param second[in] The second Polygon.
     * \return The union of both Shape
     */
    [[nodiscard]] static geometry::Shape safeUnion(const geometry::Shape& first, const geometry::Shape& second = geometry::Shape())
    {
        // The unionPolygons function can slowly remove Polygons under certain circumstances, because of rounding issues (Polygons that have a thin area).
        // This does not cause a problem when actually using it on large areas, but as influence areas (representing centerpoints) can be very thin, this does occur so this ugly
        // workaround is needed Here is an example of a Polygons object that will loose vertices when unioning, and will be gone after a few times unionPolygons was called:
        /*
            120410,83599
            120384,83643
            120399,83618
            120414,83591
            120423,83570
            120419,83580
        */

        const bool was_empty = first.empty() && second.empty();
        geometry::Shape result = first.unionPolygons(second);

        if (result.empty() && ! was_empty) // Some error occurred.
        {
            spdlog::warn("Caught an area destroying union, enlarging areas a bit.");

            // Just take the few lines we have, and offset them a tiny bit. Needs to be offsetPolylines, as offset may already have problems with the area.
            return toPolylines(first).offset(2).unionPolygons(toPolylines(second).offset(2));
        }
        return result;
    }

    /*!
     * \brief Offsets (increases the area of) a polygons object in multiple steps to ensure that it does not lag through over a given obstacle.
     * \param me[in] Shape object that has to be offset.
     * \param distance[in] The distance by which me should be offset. Expects values >=0.
     * \param collision[in] The area representing obstacles.
     * \param last_step_offset_without_check[in] The most it is allowed to offset in one step.
     * \param min_amount_offset[in] How many steps have to be done at least. As this uses round offset this increases the amount of vertices, which may be required if Shape get
     * very small. Required as arcTolerance is not exposed in offset, which should result with a similar result, benefit may be eliminated by simplifying. \param
     * min_offset_per_step Don't get below this amount of offset per step taken. Fine-tune tradeoff between speed and accuracy. \param simplifier[in] Pointer to Simplify object if
     * the offset operation also simplify the Polygon. Improves performance. \return The resulting Shape object.
     */
    [[nodiscard]] static geometry::Shape safeOffsetInc(
        const geometry::Shape& me,
        utils::coord_t distance,
        const geometry::Shape& collision,
        utils::coord_t safe_step_size,
        utils::coord_t last_step_offset_without_check,
        size_t min_amount_offset,
        utils::coord_t min_offset_per_step,
        utils::Simplify* simplifier)
    {
        bool do_final_difference = last_step_offset_without_check == 0;
        geometry::Shape ret = safeUnion(me); // Ensure sane input.
        if (distance == 0)
        {
            return (do_final_difference ? ret.difference(collision) : ret).unionPolygons();
        }
        if (safe_step_size < 0 || last_step_offset_without_check < 0)
        {
            spdlog::error("Offset increase got invalid parameter!");
            return (do_final_difference ? ret.difference(collision) : ret).unionPolygons();
        }

        utils::coord_t step_size = std::max(min_offset_per_step, safe_step_size);
        size_t steps = distance > last_step_offset_without_check ? (distance - last_step_offset_without_check) / step_size : 0;
        if (distance - steps * step_size > last_step_offset_without_check)
        {
            if ((steps + 1) * step_size <= distance)
            {
                steps++; // This will be the case when last_step_offset_without_check >= safe_step_size
            }
            else
            {
                do_final_difference = true;
            }
        }
        if (steps + (distance < last_step_offset_without_check || distance % step_size != 0) < min_amount_offset && min_amount_offset > 1) // Note: Boolean additon!
        {
            // Reduce the stepsize to ensure it is offset the required amount of times.
            step_size = distance / min_amount_offset;
            if (step_size >= safe_step_size)
            {
                // Effectively reduce last_step_offset_without_check.
                step_size = safe_step_size;
                steps = min_amount_offset;
            }
            else
            {
                steps = distance / step_size;
            }
        }
        // Offset in steps.
        for (const size_t i : ranges::views::iota(0UL, steps))
        {
            ret = ret.offset(step_size, ClipperLib::jtRound).difference(collision).unionPolygons();
            // Ensure that if many offsets are done the performance does not suffer extremely by the new vertices of jtRound.
            if (i % 10 == 7 && simplifier)
            {
                ret = simplifier->polygon(ret);
            }
        }
        ret = ret.offset(distance - steps * step_size, ClipperLib::jtRound); // Offset the remainder.
        if (simplifier)
        {
            ret = simplifier->polygon(ret);
        }

        if (do_final_difference)
        {
            ret = ret.difference(collision);
        }
        return ret.unionPolygons();
    }

    /*!
     * \brief Moves the points of a Polyline outside of a given area, it the distance is smaller than max_allowed_distance
     *
     * \param polylines[in] The polyline object from which the lines are moved.
     * \param area[in] The area the points are moved out of.
     * \param max_allowed_distance[in] The maximum distance a point may be moved. If not possible the point will be moved as far as possible in the direction of the outside of the
     * provided area. \return A Polyline object containing the moved points.
     */
    [[nodiscard]] static geometry::OpenLinesSet movePointsOutside(const geometry::OpenLinesSet& polylines, const geometry::Shape& area, utils::coord_t max_allowed_distance)
    {
        geometry::OpenLinesSet result;

        for (const geometry::OpenPolyline& line : polylines)
        {
            geometry::OpenPolyline next_line;
            for (const geometry::Point2LL& p : line)
            {
                if (area.inside(p))
                {
                    geometry::Point2LL next_outside = p;
                    utils::PolygonUtils::moveOutside(area, next_outside);
                    if (geometry::vSize2(p - next_outside) < max_allowed_distance * max_allowed_distance)
                    {
                        next_line.push_back(next_outside);
                    }
                    else // move point as far as allowed.
                    {
                        double max_partial_move_proportion = double(max_allowed_distance) / double(geometry::vSize(p - next_outside));
                        next_outside = p + (next_outside - p) * max_partial_move_proportion;
                        next_line.push_back(next_outside);
                    }
                }
                else
                {
                    next_line.push_back(p);
                }
            }
            if (next_line.size() > 0)
            {
                result.push_back(next_line);
            }
        }

        return result;
    }

    [[nodiscard]] static utils::VariableWidthLines polyLineToVWL(const geometry::Shape& polylines, utils::coord_t line_width)
    {
        utils::VariableWidthLines result;
        for (auto path : polylines)
        {
            utils::ExtrusionLine vwl_line(1, true);

            for (geometry::Point2LL p : path)
            {
                vwl_line.emplace_back(p, line_width, 1);
            }
            result.emplace_back(vwl_line);
        }
        return result;
    }
};

} // namespace curaengine
