//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <algorithm> //For std::sort.

#include "infill.h"
#include "functional"
#include "utils/polygonUtils.h"
#include "utils/logoutput.h"
#include "utils/UnionFind.h"

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
static inline int computeScanSegmentIdx(int x, int line_width);


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

namespace cura {

void Infill::generate(Polygons& result_polygons, Polygons& result_lines, const SpaceFillingTreeFill* cross_fill_pattern, const SliceMeshStorage* mesh)
{
    if (in_outline.size() == 0) return;
    if (line_distance == 0) return;
    switch(pattern)
    {
    case EFillMethod::GRID:
        generateGridInfill(result_lines);
        break;
    case EFillMethod::LINES:
        generateLineInfill(result_lines, line_distance, fill_angle, 0);
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
        generateConcentricInfill(result_polygons, line_distance);
        break;
    case EFillMethod::CONCENTRIC_3D:
        generateConcentric3DInfill(result_polygons);
        break;
    case EFillMethod::ZIG_ZAG:
        generateZigZagInfill(result_lines, line_distance, fill_angle);
        break;
    case EFillMethod::CUBICSUBDIV:
        if (!mesh)
        {
            logError("Cannot generate Cubic Subdivision infill without a mesh!\n");
            break;
        }
        generateCubicSubDivInfill(result_lines, *mesh);
        break;
    case EFillMethod::CROSS:
    case EFillMethod::CROSS_3D:
        if (!cross_fill_pattern)
        {
            logError("Cannot generate Cross infill without a pregenerated cross fill pattern!\n");
            break;
        }
        generateCrossInfill(*cross_fill_pattern, result_polygons, result_lines);
        break;
    default:
        logError("Fill pattern has unknown value.\n");
        break;
    }

    if (zig_zaggify)
    {
        connectLines(result_lines);
    }
    crossings_on_line.clear();
}

void Infill::generateConcentricInfill(Polygons& result, int inset_value)
{
    Polygons first_concentric_wall = in_outline.offset(outline_offset - line_distance + infill_line_width / 2); // - infill_line_width / 2 cause generateConcentricInfill expects [outline] to be the outer most polygon instead of the outer outline

    if (perimeter_gaps)
    {
        const Polygons inner = first_concentric_wall.offset(infill_line_width / 2 + perimeter_gaps_extra_offset);
        const Polygons gaps_here = in_outline.difference(inner);
        perimeter_gaps->add(gaps_here);
    }
    generateConcentricInfill(first_concentric_wall, result, inset_value);
}

void Infill::generateConcentricInfill(Polygons& first_concentric_wall, Polygons& result, int inset_value)
{
    result.add(first_concentric_wall);
    Polygons* prev_inset = &first_concentric_wall;
    Polygons next_inset;
    while (prev_inset->size() > 0)
    {
        next_inset = prev_inset->offset(-inset_value);
        result.add(next_inset);
        if (perimeter_gaps)
        {
            const Polygons outer = prev_inset->offset(-infill_line_width / 2 - perimeter_gaps_extra_offset);
            const Polygons inner = next_inset.offset(infill_line_width / 2);
            const Polygons gaps_here = outer.difference(inner);
            perimeter_gaps->add(gaps_here);
        }
        prev_inset = &next_inset;
    }
    std::reverse(std::begin(result), std::end(result));
}

void Infill::generateConcentric3DInfill(Polygons& result)
{
    coord_t period = line_distance * 2;
    coord_t shift = int64_t(one_over_sqrt_2 * z) % period;
    shift = std::min(shift, period - shift); // symmetry due to the fact that we are applying the shift in both directions
    shift = std::min(shift, period / 2 - infill_line_width / 2); // don't put lines too close to each other
    shift = std::max(shift, infill_line_width / 2); // don't put lines too close to each other
    Polygons first_wall;
    // in contrast to concentric infill we dont do "- infill_line_width / 2" cause this is already handled by the max two lines above
    first_wall = in_outline.offset(outline_offset - shift);
    generateConcentricInfill(first_wall, result, period);
    first_wall = in_outline.offset(outline_offset - period + shift);
    generateConcentricInfill(first_wall, result, period);
}

void Infill::generateGridInfill(Polygons& result)
{
    generateLineInfill(result, line_distance, fill_angle, 0);
    generateLineInfill(result, line_distance, fill_angle + 90, 0);
}

void Infill::generateCubicInfill(Polygons& result)
{
    int64_t shift = one_over_sqrt_2 * z;
    generateLineInfill(result, line_distance, fill_angle, shift);
    generateLineInfill(result, line_distance, fill_angle + 120, shift);
    generateLineInfill(result, line_distance, fill_angle + 240, shift);
}

void Infill::generateTetrahedralInfill(Polygons& result)
{
    generateHalfTetrahedralInfill(0.0, 0, result);
    generateHalfTetrahedralInfill(0.0, 90, result);
}

void Infill::generateQuarterCubicInfill(Polygons& result)
{
    generateHalfTetrahedralInfill(0.0, 0, result);
    generateHalfTetrahedralInfill(0.5, 90, result);
}

void Infill::generateHalfTetrahedralInfill(float pattern_z_shift, int angle_shift, Polygons& result)
{
    coord_t period = line_distance * 2;
    coord_t shift = int64_t(one_over_sqrt_2 * (z + pattern_z_shift * period * 2)) % period;
    shift = std::min(shift, period - shift); // symmetry due to the fact that we are applying the shift in both directions
    shift = std::min(shift, period / 2 - infill_line_width / 2); // don't put lines too close to each other
    shift = std::max(shift, infill_line_width / 2); // don't put lines too close to each other
    generateLineInfill(result, period, fill_angle + angle_shift, shift);
    generateLineInfill(result, period, fill_angle + angle_shift, -shift);
}

void Infill::generateTriangleInfill(Polygons& result)
{
    generateLineInfill(result, line_distance, fill_angle, 0);
    generateLineInfill(result, line_distance, fill_angle + 60, 0);
    generateLineInfill(result, line_distance, fill_angle + 120, 0);
}

void Infill::generateTrihexagonInfill(Polygons& result)
{
    generateLineInfill(result, line_distance, fill_angle, 0);
    generateLineInfill(result, line_distance, fill_angle + 60, 0);
    generateLineInfill(result, line_distance, fill_angle + 120, line_distance / 2);
}

void Infill::generateCubicSubDivInfill(Polygons& result, const SliceMeshStorage& mesh)
{
    Polygons uncropped;
    mesh.base_subdiv_cube->generateSubdivisionLines(z, uncropped);
    addLineSegmentsInfill(result, uncropped);
}

void Infill::generateCrossInfill(const SpaceFillingTreeFill& cross_fill_pattern, Polygons& result_polygons, Polygons& result_lines)
{
    if (zig_zaggify)
    {
        outline_offset += -infill_line_width / 2;
    }
    coord_t shift = line_distance / 2;
    bool use_odd_in_junctions = false;
    bool use_odd_out_junctions = false;
    if (pattern == EFillMethod::CROSS_3D)
    {
        coord_t period = line_distance * 2;
        shift = z % period;
        shift = std::min(shift, period - shift); // symmetry due to the fact that we are applying the shift in both directions
        shift = std::min(shift, period / 2 - infill_line_width / 2); // don't put lines too close to each other
        shift = std::max(shift, infill_line_width / 2); // don't put lines too close to each other

        use_odd_in_junctions = ((z + period / 2) / period) % 2 == 1; // change junction halfway in between each period when the in-junctions occur
        use_odd_out_junctions = (z / period) % 2 == 1; // out junctions occur halfway at each periods
    }
    Polygons outline = in_outline.offset(outline_offset);
    cross_fill_pattern.generate(outline, shift, zig_zaggify, fill_angle, apply_pockets_alternatingly, use_odd_in_junctions, use_odd_out_junctions, pocket_size, result_polygons, result_lines);
}

void Infill::addLineSegmentsInfill(Polygons& result, Polygons& input)
{
    ClipperLib::PolyTree interior_segments_tree = in_outline.lineSegmentIntersection(input);
    ClipperLib::Paths interior_segments;
    ClipperLib::OpenPathsFromPolyTree(interior_segments_tree, interior_segments);
    for (uint64_t idx = 0; idx < interior_segments.size(); idx++)
    {
        result.addLine(interior_segments[idx][0], interior_segments[idx][1]);
    }
}

void Infill::addLineInfill(Polygons& result, const PointMatrix& rotation_matrix, const int scanline_min_idx, const int line_distance, const AABB boundary, std::vector<std::vector<int64_t>>& cut_list, int64_t shift)
{
    auto compare_int64_t = [](const void* a, const void* b)
    {
        int64_t n = (*(int64_t*)a) - (*(int64_t*)b);
        if (n < 0)
        {
            return -1;
        }
        if (n > 0)
        {
            return 1;
        }
        return 0;
    };

    unsigned int scanline_idx = 0;
    for(int64_t x = scanline_min_idx * line_distance + shift; x < boundary.max.X; x += line_distance)
    {
        if (scanline_idx >= cut_list.size())
        {
            break;
        }
        std::vector<int64_t>& crossings = cut_list[scanline_idx];
        qsort(crossings.data(), crossings.size(), sizeof(int64_t), compare_int64_t);
        for(unsigned int crossing_idx = 0; crossing_idx + 1 < crossings.size(); crossing_idx += 2)
        {
            if (crossings[crossing_idx + 1] - crossings[crossing_idx] < infill_line_width / 5)
            { // segment is too short to create infill
                continue;
            }
            result.addLine(rotation_matrix.unapply(Point(x, crossings[crossing_idx])), rotation_matrix.unapply(Point(x, crossings[crossing_idx + 1])));
        }
        scanline_idx += 1;
    }
}

int64_t Infill::getShiftOffsetFromInfillOriginAndRotation(const double& infill_rotation)
{
    if (infill_origin.X != 0 || infill_origin.Y != 0)
    {
        const double rotation_rads = infill_rotation * M_PI / 180;
        return infill_origin.X * std::cos(rotation_rads) - infill_origin.Y * std::sin(rotation_rads);
    }
    return 0;
}

void Infill::generateLineInfill(Polygons& result, int line_distance, const double& infill_rotation, int64_t shift)
{
    shift += getShiftOffsetFromInfillOriginAndRotation(infill_rotation);
    PointMatrix rotation_matrix(infill_rotation);
    NoZigZagConnectorProcessor lines_processor(rotation_matrix, result);
    bool connected_zigzags = false;
    generateLinearBasedInfill(outline_offset, result, line_distance, rotation_matrix, lines_processor, connected_zigzags, shift);
}


void Infill::generateZigZagInfill(Polygons& result, const int line_distance, const double& infill_rotation)
{
    const int64_t shift = getShiftOffsetFromInfillOriginAndRotation(infill_rotation);

    PointMatrix rotation_matrix(infill_rotation);
    ZigzagConnectorProcessor zigzag_processor(rotation_matrix, result, use_endpieces, connected_zigzags, skip_some_zags, zag_skip_count, minimum_zag_line_length);
    generateLinearBasedInfill(outline_offset - infill_line_width / 2, result, line_distance, rotation_matrix, zigzag_processor, connected_zigzags, shift);
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
void Infill::generateLinearBasedInfill(const int outline_offset, Polygons& result, const int line_distance, const PointMatrix& rotation_matrix, ZigzagConnectorProcessor& zigzag_connector_processor, const bool connected_zigzags, int64_t extra_shift)
{
    if (line_distance == 0)
    {
        return;
    }
    if (in_outline.size() == 0)
    {
        return;
    }

    int shift = extra_shift + this->shift;

    Polygons outline;
    if (outline_offset != 0)
    {
        outline = in_outline.offset(outline_offset);
        if (perimeter_gaps)
        {
            perimeter_gaps->add(in_outline.difference(outline.offset(infill_line_width / 2 + perimeter_gaps_extra_offset)));
        }
    }
    else
    {
        outline = in_outline;
    }

    outline = outline.offset(infill_overlap);

    if (outline.size() == 0)
    {
        return;
    }
    //TODO: Currently we find the outline every time for each rotation.
    //We should compute it only once and rotate that accordingly.
    //We'll also have the guarantee that they have the same size every time.
    //Currently we assume that the above operations are all rotation-invariant,
    //which they aren't if vertices fall on the same coordinate due to rounding.
    crossings_on_line.resize(outline.size()); //One for each polygon.

    outline.applyMatrix(rotation_matrix);

    if (shift < 0)
    {
        shift = line_distance - (-shift) % line_distance;
    }
    else
    {
        shift = shift % line_distance;
    }

    AABB boundary(outline);

    int scanline_min_idx = computeScanSegmentIdx(boundary.min.X - shift, line_distance);
    int line_count = computeScanSegmentIdx(boundary.max.X - shift, line_distance) + 1 - scanline_min_idx;

    std::vector<std::vector<int64_t> > cut_list; // mapping from scanline to all intersections with polygon segments

    for(int scanline_idx = 0; scanline_idx < line_count; scanline_idx++)
    {
        cut_list.push_back(std::vector<int64_t>());
    }

    for(size_t poly_idx = 0; poly_idx < outline.size(); poly_idx++)
    {
        PolygonRef poly = outline[poly_idx];
        crossings_on_line[poly_idx].resize(poly.size()); //One for each line in this polygon.
        Point p0 = poly.back();
        zigzag_connector_processor.registerVertex(p0); // always adds the first point to ZigzagConnectorProcessorEndPieces::first_zigzag_connector when using a zigzag infill type

        //When we find crossings, keep track of which crossing belongs to which scanline and to which polygon line segment.
        //Then we can later join two crossings together to form lines and still know what polygon line segments that infill line connected to.
        struct Crossing
        {
            Crossing(Point coordinate, size_t vertex_index): coordinate(coordinate), vertex_index(vertex_index) {};
            Point coordinate;
            size_t vertex_index;
            bool operator <(const Crossing& other) const //Crossings will be ordered by their X coordinate so that they get ordered along the scanline.
            {
                return coordinate.X < other.coordinate.X;
            }
        };
        std::vector<std::vector<Crossing>> crossings_per_scanline;
        const int min_scanline_index = computeScanSegmentIdx(poly.min().X - shift, line_distance) + 1;
        const int max_scanline_index = computeScanSegmentIdx(poly.max().X - shift, line_distance) + 1;
        crossings_per_scanline.resize(max_scanline_index - min_scanline_index);

        for(size_t point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            Point p1 = poly[point_idx];
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
                scanline_idx1 = computeScanSegmentIdx(p1.X - shift, line_distance); // -1 cause the vertex point is handled in the next segment (or not in the case which looks like >)
            }
            else
            {
                direction = -1;
                scanline_idx0 = computeScanSegmentIdx(p0.X - shift, line_distance); // -1 cause the vertex point is handled in the previous segment (or not in the case which looks like >)
                scanline_idx1 = computeScanSegmentIdx(p1.X - shift, line_distance) + 1; // + 1 cause we don't cross the scanline of the first scan segment
            }

            for(int scanline_idx = scanline_idx0; scanline_idx != scanline_idx1 + direction; scanline_idx += direction)
            {
                int x = scanline_idx * line_distance + shift;
                int y = p1.Y + (p0.Y - p1.Y) * (x - p1.X) / (p0.X - p1.X);
                assert(scanline_idx - scanline_min_idx >= 0 && scanline_idx - scanline_min_idx < int(cut_list.size()) && "reading infill cutlist index out of bounds!");
                cut_list[scanline_idx - scanline_min_idx].push_back(y);
                Point scanline_linesegment_intersection(x, y);
                zigzag_connector_processor.registerScanlineSegmentIntersection(scanline_linesegment_intersection, scanline_idx);
                crossings_per_scanline[scanline_idx].emplace_back(scanline_linesegment_intersection, point_idx);
            }
            zigzag_connector_processor.registerVertex(p1);
            p0 = p1;
        }
        zigzag_connector_processor.registerPolyFinished();

        //Gather all crossings per scanline and find out which crossings belong together, then store them in crossings_on_line.
        for (int scanline_index = min_scanline_index; scanline_index < max_scanline_index; scanline_index++)
        {
            std::sort(crossings_per_scanline[scanline_index].begin(), crossings_per_scanline[scanline_index].end()); //Sorts them by X coordinate.
            for (size_t crossing_index = 0; crossing_index < crossings_per_scanline[scanline_index].size() - 1; crossing_index += 2) //Combine each 2 subsequent crossings together.
            {
                const Crossing& first = crossings_per_scanline[scanline_index][crossing_index];
                const Crossing& second = crossings_per_scanline[scanline_index][crossing_index + 1];
                all_infill_lines.emplace_back(first.coordinate, first.vertex_index, second.coordinate, second.vertex_index);
                //Put the same line segment in the data structure twice: Once for each of the polygon line segment that it crosses.
                crossings_on_line[poly_idx][first.vertex_index].push_back(&all_infill_lines.back());
                crossings_on_line[poly_idx][second.vertex_index].push_back(&all_infill_lines.back());
            }
        }
    }

    if (cut_list.size() == 0)
    {
        return;
    }
    if (connected_zigzags && cut_list.size() == 1 && cut_list[0].size() <= 2)
    {
        return;  // don't add connection if boundary already contains whole outline!
    }

    addLineInfill(result, rotation_matrix, scanline_min_idx, line_distance, boundary, cut_list, shift);
}

void Infill::connectLines(Polygons& result_lines)
{
    if (result_lines.empty()) //Too small area or a pattern that generates polygons instead of lines.
    {
        return;
    }
    if (pattern == EFillMethod::ZIG_ZAG) //TODO: For now, we skip ZigZag because it has its own algorithms. Eventually we want to replace all that with the new algorithm.
    {
        return;
    }

    //TODO: We're reconstructing the outline here. We should store it and compute it only once.
    Polygons outline = in_outline.offset(outline_offset + infill_overlap);

    UnionFind<InfillLineSegment*> connected_lines; //Keeps track of which lines are connected to which.
    std::vector<InfillLineSegment> connecting_lines; //Keeps all connecting lines in memory, as to not invalidate the pointers.
    connecting_lines.reserve(result_lines.size());

    for (size_t polygon_index = 0; polygon_index < outline.size(); polygon_index++)
    {
        if (outline[polygon_index].empty())
        {
            continue;
        }

        InfillLineSegment* previous_crossing = nullptr; //The crossing that we should connect to. If nullptr, we have been skipping until we find the next crossing.
        Point vertex_before = outline[polygon_index].back();
        for (size_t vertex_index = 0; vertex_index < outline[polygon_index].size(); vertex_index++)
        {
            Point vertex_after = outline[polygon_index][vertex_index];

            //Sort crossings on every line by how far they are from their initial point.
            struct CompareByDistance
            {
                CompareByDistance(Point to_point, size_t vertex_index): to_point(to_point), vertex_index(vertex_index) {};
                Point to_point; //The distance to this point is compared.
                size_t vertex_index; //The vertex indicating a line segment. This determines which endpoint of each line should be used.
                inline bool operator ()(InfillLineSegment*& left_hand_side, InfillLineSegment*& right_hand_side) const
                {
                    //Find the two endpoints that are relevant.
                    const Point left_hand_point = (left_hand_side->start_segment == vertex_index) ? left_hand_side->start : left_hand_side->end;
                    const Point right_hand_point = (right_hand_side->start_segment == vertex_index) ? right_hand_side->start : right_hand_side->end;
                    return vSize(left_hand_point - to_point) < vSize(right_hand_point - to_point);
                }
            };
            std::sort(crossings_on_line[polygon_index][vertex_index].begin(), crossings_on_line[polygon_index][vertex_index].end(), CompareByDistance(vertex_before, vertex_index));

            for (InfillLineSegment* crossing : crossings_on_line[polygon_index][vertex_index])
            {
                if (!previous_crossing) //If we're not yet drawing, then we have been trying to find the next vertex. We found it! Let's start drawing.
                {
                    previous_crossing = crossing;
                }
                else
                {
                    const size_t crossing_handle = connected_lines.find(crossing);
                    const size_t previous_crossing_handle = connected_lines.find(previous_crossing);
                    if (crossing_handle == previous_crossing_handle) //These two infill lines are already connected. Don't create a loop now. Continue connecting with the next crossing.
                    {
                        continue;
                    }

                    //Join two infill lines together with a connecting line.
                    //Here the InfillLineSegments function as a linked list, so that they can easily be joined.
                    const Point previous_point = (previous_crossing->start_segment == vertex_index) ? previous_crossing->start : previous_crossing->end;
                    const Point next_point = (crossing->start_segment == vertex_index) ? crossing->start : crossing->end;
                    connecting_lines.emplace_back(previous_point, vertex_index, next_point, vertex_index);
                    InfillLineSegment* new_segment = &connecting_lines.back();
                    new_segment->previous = previous_crossing;
                    if (previous_crossing->start_segment == vertex_index)
                    {
                        previous_crossing->previous = new_segment;
                    }
                    else
                    {
                        previous_crossing->next = new_segment;
                    }
                    new_segment->next = crossing;
                    if (crossing->start_segment == vertex_index)
                    {
                        crossing->previous = new_segment;
                    }
                    else
                    {
                        crossing->next = new_segment;
                    }
                    crossing->previous = new_segment;
                    connected_lines.unite(crossing_handle, previous_crossing_handle);
                    previous_crossing = nullptr;
                }
            }

            //Upon going to the next vertex, if we're drawing, put an extra vertex in our infill lines.
            if (previous_crossing)
            {
                connecting_lines.emplace_back(previous_crossing->end, vertex_index, vertex_after, vertex_index + 1);
                connecting_lines.back().previous = previous_crossing;
                previous_crossing->next = &connecting_lines.back();
                previous_crossing = &connecting_lines.back();
            }

            vertex_before = vertex_after;
        }
    }
}

bool Infill::InfillLineSegment::operator ==(const InfillLineSegment& other) const
{
    return start == other.start && end == other.end;
}

size_t Infill::HashInfillLineSegment::operator()(const InfillLineSegment& infill_line_segment) const
{
    return std::hash<Point>()(infill_line_segment.start) ^ (std::hash<Point>()(infill_line_segment.end) << 7);
}

}//namespace cura