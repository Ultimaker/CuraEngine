/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "infill.h"
#include "functional"
#include "utils/polygonUtils.h"
#include "utils/logoutput.h"

#define SQRT2MUL(x) ((30547*(x))/21600)
#define OCTSLEN(x) ((43200*(x))/73747)
#define OCTDLEN(x) ((21600*(x))/30547)

namespace cura {

int Infill::computeScanSegmentIdx(int x, int line_width)
{
    if (x < 0)
    {
        return (x + 1) / line_width - 1;
        // - 1 because -1 belongs to scansegment -1
        // + 1 because -line_width belongs to scansegment -1
    }
    return x / line_width;
}

void Infill::generate(Polygons& result_polygons, Polygons& result_lines)
{
    if (in_outline.size() == 0) return;
    if (line_distance == 0) return;
    const Polygons* outline = &in_outline;
    Polygons outline_offsetted;
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
    case EFillMethod::TRIANGLES:
        generateTriangleInfill(result_lines);
        break;
    case EFillMethod::CONCENTRIC:
        outline_offsetted = in_outline.offset(outline_offset - infill_line_width / 2); // - infill_line_width / 2 cause generateConcentricInfill expects [outline] to be the outer most polygon instead of the outer outline 
        outline = &outline_offsetted;
        generateConcentricInfill(*outline, result_polygons, line_distance);
        break;
    case EFillMethod::ZIG_ZAG:
        generateZigZagInfill(result_lines, line_distance, fill_angle, connected_zigzags, use_endpieces);
        break;
    case EFillMethod::TRUNCATED_OCTAHEDRON:
        generateTroctInfill(result_lines);
        break;
    default:
        logError("Fill pattern has unknown value.\n");
        break;
    }
}

void Infill::generateTroctInfill(Polygons& result)
  {
    int extrusionWidth = infill_line_width;
    int lineSpacing = line_distance;
    int infillOverlap = infill_overlap;
    double rotation = fill_angle;
    int posZ = z;

    Polygons outline = in_outline.offset(extrusionWidth * infillOverlap / 100);
    PointMatrix matrix(rotation);
    outline.applyMatrix(matrix);
    AABB boundary(outline);

    // ignore infill for areas smaller than line spacing
    if((abs(boundary.min.X - boundary.max.X) + abs(boundary.min.Y - boundary.max.Y)) < lineSpacing){
      return;
    }

    // fix to normalise against diagonal infill
    lineSpacing = lineSpacing * 2;

    uint64_t Zscale = SQRT2MUL(lineSpacing);

    int offset = abs(posZ % (Zscale) - (Zscale/2)) - (Zscale/4);
    boundary.min.X = ((boundary.min.X / lineSpacing) - 1) * lineSpacing;
    boundary.min.Y = ((boundary.min.Y / lineSpacing) - 1) * lineSpacing;

    unsigned int lineCountX = (boundary.max.X - boundary.min.X + (lineSpacing - 1)) / lineSpacing;
    unsigned int lineCountY = (boundary.max.Y - boundary.min.Y + (lineSpacing - 1)) / lineSpacing;
    int rtMod = int(rotation / 90) % 2;
    // with an odd number of lines, sides need to be swapped around
    if(rtMod == 1){
      rtMod = (lineCountX + int(rotation / 90)) % 2;
    }

    // draw non-horizontal walls of octohedrons
    Polygons po;
    PolygonRef p = po.newPoly();
    for(unsigned int ly=0; ly < lineCountY;){
      for(size_t it = 0; it < 2; ly++, it++){
        int side = (2*((ly + it + rtMod) % 2) - 1);
        int y = (ly * lineSpacing) + boundary.min.Y + lineSpacing / 2 - (offset/2 * side);
        int x = boundary.min.X-(offset/2);
        if(it == 1){
          x = (lineCountX * (lineSpacing)) + boundary.min.X + lineSpacing / 2 - (offset/2);
        }
        p.add(Point(x,y));
        for(unsigned int lx=0; lx < lineCountX; lx++){
          if(it == 1){
            side = (2*((lx + ly + it + rtMod + lineCountX) % 2) - 1);
            y = (ly * lineSpacing) + boundary.min.Y + lineSpacing / 2 + (offset/2 * side);
            x = ((lineCountX - lx - 1) * lineSpacing) + boundary.min.X + lineSpacing / 2;
            p.add(Point(x+lineSpacing-abs(offset/2), y));
            p.add(Point(x+abs(offset/2), y));
          } else {
            side = (2*((lx + ly + it + rtMod) % 2) - 1);
            y = (ly * lineSpacing) + boundary.min.Y + lineSpacing / 2 + (offset/2 * side);
            x = (lx * lineSpacing) + boundary.min.X + lineSpacing / 2;
            p.add(Point(x+abs(offset/2), y));
            p.add(Point(x+lineSpacing-abs(offset/2), y));
          }
        }
        x = (lineCountX * lineSpacing) + boundary.min.X + lineSpacing / 2 - (offset/2);
        if(it == 1){
          x = boundary.min.X-(offset/2);
        }
        y = (ly * lineSpacing) + boundary.min.Y + lineSpacing / 2 - (offset/2 * side);
        p.add(Point(x,y));
      }
    }
    // Generate tops / bottoms of octohedrons
    if(abs((abs(offset) - Zscale/4)) < (extrusionWidth/2)){
      uint64_t startLine = (offset < 0) ? 0 : 1;
      uint64_t coverWidth = OCTSLEN(lineSpacing);
      std::vector<Point> points;
      for(size_t xi = 0; xi < (lineCountX+1); xi++){
        for(size_t yi = 0; yi < (lineCountY); yi += 2){
          points.push_back(Point(boundary.min.X + OCTDLEN(lineSpacing)
                                 + (xi - startLine + rtMod) * lineSpacing,
                                 boundary.min.Y + OCTDLEN(lineSpacing)
                                 + (yi + (xi%2)) * lineSpacing
                                 + extrusionWidth/2));
        }
      }
      uint64_t order = 0;
      for(Point pp : points){
        PolygonRef p = po.newPoly();
        for(size_t yi = 0; yi <= coverWidth; yi += extrusionWidth) {
          if(order == 0){
            p.add(Point(pp.X, pp.Y + yi));
            p.add(Point(pp.X + coverWidth + extrusionWidth, pp.Y + yi));
          } else {
            p.add(Point(pp.X + coverWidth + extrusionWidth, pp.Y + yi));
            p.add(Point(pp.X, pp.Y + yi));
          }
          order = (order + 1) % 2;
        }
      }
    }
    // intersect with outline polygon(s)
    Polygons pi = po.intersection(outline);
    // Hack to add intersection to result. There doesn't seem
    // to be a direct way to do this
    for(unsigned int polyNr=0; polyNr < pi.size(); polyNr++) {
      PolygonRef p = result.newPoly(); //  = result.newPoly()
      for(unsigned int i=0; i < pi[polyNr].size(); i++) {
        Point p0 = pi[polyNr][i];
        p.add(matrix.unapply(Point(p0.X,p0.Y)));
      }
    }
}

void Infill::generateConcentricInfill(Polygons outline, Polygons& result, int inset_value)
{
    while(outline.size() > 0)
    {
        result.add(outline);
        outline = outline.offset(-inset_value);
    } 
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
    int shift = int64_t(one_over_sqrt_2 * z) % line_distance;
    shift = std::min(shift, line_distance - shift); // symmetry due to the fact that we are applying the shift in both directions
    shift = std::min(shift, line_distance / 2 - infill_line_width / 2); // don't put lines too close to each other
    shift = std::max(shift, infill_line_width / 2); // don't put lines too close to each other
    generateLineInfill(result, line_distance, fill_angle, shift);
    generateLineInfill(result, line_distance, fill_angle, -shift);
    generateLineInfill(result, line_distance, fill_angle + 90, shift);
    generateLineInfill(result, line_distance, fill_angle + 90, -shift);
}

void Infill::generateTriangleInfill(Polygons& result)
{
    generateLineInfill(result, line_distance, fill_angle, 0);
    generateLineInfill(result, line_distance, fill_angle + 60, 0);
    generateLineInfill(result, line_distance, fill_angle + 120, 0);
}

void Infill::addLineInfill(Polygons& result, const PointMatrix& rotation_matrix, const int scanline_min_idx, const int line_distance, const AABB boundary, std::vector<std::vector<int64_t>>& cut_list, int64_t shift)
{
    auto addLine = [&](Point from, Point to)
    {
        PolygonRef p = result.newPoly();
        p.add(rotation_matrix.unapply(from));
        p.add(rotation_matrix.unapply(to));
    };

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

    int scanline_idx = 0;
    for(int64_t x = scanline_min_idx * line_distance + shift; x < boundary.max.X; x += line_distance)
    {
        std::vector<int64_t>& crossings = cut_list[scanline_idx];
        qsort(crossings.data(), crossings.size(), sizeof(int64_t), compare_int64_t);
        for(unsigned int crossing_idx = 0; crossing_idx + 1 < crossings.size(); crossing_idx += 2)
        {
            if (crossings[crossing_idx + 1] - crossings[crossing_idx] < infill_line_width / 5)
            { // segment is too short to create infill
                continue;
            }
            addLine(Point(x, crossings[crossing_idx]), Point(x, crossings[crossing_idx + 1]));
        }
        scanline_idx += 1;
    }
}

void Infill::generateLineInfill(Polygons& result, int line_distance, const double& fill_angle, int64_t shift)
{
    PointMatrix rotation_matrix(fill_angle);
    NoZigZagConnectorProcessor lines_processor(rotation_matrix, result);
    bool connected_zigzags = false;
    generateLinearBasedInfill(outline_offset, result, line_distance, rotation_matrix, lines_processor, connected_zigzags, shift);
}


void Infill::generateZigZagInfill(Polygons& result, const int line_distance, const double& fill_angle, const bool connected_zigzags, const bool use_endpieces)
{

    PointMatrix rotation_matrix(fill_angle);
    if (use_endpieces)
    {
        if (connected_zigzags)
        {
            ZigzagConnectorProcessorConnectedEndPieces zigzag_processor(rotation_matrix, result);
            generateLinearBasedInfill(outline_offset - infill_line_width / 2, result, line_distance, rotation_matrix, zigzag_processor, connected_zigzags, 0);
        }
        else
        {
            ZigzagConnectorProcessorDisconnectedEndPieces zigzag_processor(rotation_matrix, result);
            generateLinearBasedInfill(outline_offset - infill_line_width / 2, result, line_distance, rotation_matrix, zigzag_processor, connected_zigzags, 0);
        }
    }
    else 
    {
        ZigzagConnectorProcessorNoEndPieces zigzag_processor(rotation_matrix, result);
        generateLinearBasedInfill(outline_offset - infill_line_width / 2, result, line_distance, rotation_matrix, zigzag_processor, connected_zigzags, 0);
    }
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

    for(unsigned int poly_idx = 0; poly_idx < outline.size(); poly_idx++)
    {
        PolygonRef poly = outline[poly_idx];
        Point p0 = poly.back();
        zigzag_connector_processor.registerVertex(p0); // always adds the first point to ZigzagConnectorProcessorEndPieces::first_zigzag_connector when using a zigzag infill type
        for(unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            Point p1 = poly[point_idx];
            if (p1.X == p0.X)
            {
                zigzag_connector_processor.registerVertex(p1); 
                // TODO: how to make sure it always adds the shortest line? (in order to prevent overlap with the zigzag connectors)
                // note: this is already a problem for normal infill, but hasn't really cothered anyone so far.
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
                zigzag_connector_processor.registerScanlineSegmentIntersection(scanline_linesegment_intersection, scanline_idx % 2 == 0);
            }
            zigzag_connector_processor.registerVertex(p1);
            p0 = p1;
        }
        zigzag_connector_processor.registerPolyFinished();
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

}//namespace cura
