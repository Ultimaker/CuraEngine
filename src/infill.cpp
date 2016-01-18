/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "infill.h"
#include "functional"
#include "utils/polygonUtils.h"
#include "utils/AABB.h"
#include "utils/logoutput.h"

namespace cura {

void Infill::generate(Polygons& result_polygons, Polygons& result_lines, Polygons* in_between)
{
    if (in_outline.size() == 0) return;
    if (line_distance == 0) return;
    const Polygons* outline = &in_outline;
    Polygons outline_offsetted;
    switch(pattern)
    {
    case EFillMethod::GRID:
        generateGridInfill(in_outline, outlineOffset, result_lines, extrusion_width, line_distance * 2, infill_overlap, fill_angle);
        break;
    case EFillMethod::LINES:
        generateLineInfill(in_outline, outlineOffset, result_lines, extrusion_width, line_distance, infill_overlap, fill_angle);
        break;
    case EFillMethod::TRIANGLES:
        generateTriangleInfill(in_outline, outlineOffset, result_lines, extrusion_width, line_distance * 3, infill_overlap, fill_angle);
        break;
    case EFillMethod::CONCENTRIC:
        PolygonUtils::offsetSafe(in_outline, outlineOffset - extrusion_width / 2, extrusion_width, outline_offsetted, false); // - extrusion_width / 2 cause generateConcentricInfill expects [outline] to be the outer most polygon instead of the outer outline 
        outline = &outline_offsetted;
        if (abs(extrusion_width - line_distance) < 10)
        {
            generateConcentricInfillDense(*outline, result_polygons, in_between, extrusion_width, avoidOverlappingPerimeters);
        }
        else 
        {
            generateConcentricInfill(*outline, result_polygons, line_distance);
        }
        break;
    case EFillMethod::ZIG_ZAG:
        if (outlineOffset != 0)
        {
            PolygonUtils::offsetSafe(in_outline, outlineOffset, extrusion_width, outline_offsetted, avoidOverlappingPerimeters);
            outline = &outline_offsetted;
        }
        generateZigZagInfill(*outline, result_lines, extrusion_width, line_distance, infill_overlap, fill_angle, connect_zigzags, use_endPieces);
        break;
    default:
        logError("Fill pattern has unknown value.\n");
        break;
    }
}

    
      
void generateConcentricInfillDense(Polygons outline, Polygons& result, Polygons* in_between, int extrusionWidth, bool avoidOverlappingPerimeters)
{
    while(outline.size() > 0)
    {
        for (unsigned int polyNr = 0; polyNr < outline.size(); polyNr++)
        {
            PolygonRef r = outline[polyNr];
            result.add(r);
        }
        Polygons next_outline;
        PolygonUtils::offsetExtrusionWidth(outline, true, extrusionWidth, next_outline, in_between, avoidOverlappingPerimeters);
        outline = next_outline;
    } 

}

void generateConcentricInfill(Polygons outline, Polygons& result, int inset_value)
{
    while(outline.size() > 0)
    {
        for (unsigned int polyNr = 0; polyNr < outline.size(); polyNr++)
        {
            PolygonRef r = outline[polyNr];
            result.add(r);
        }
        outline = outline.offset(-inset_value);
    } 
}


void generateGridInfill(const Polygons& in_outline, int outlineOffset, Polygons& result,
                        int extrusionWidth, int lineSpacing, double infillOverlap,
                        double rotation)
{
    generateLineInfill(in_outline, outlineOffset, result, extrusionWidth, lineSpacing,
                       infillOverlap, rotation);
    generateLineInfill(in_outline, outlineOffset, result, extrusionWidth, lineSpacing,
                       infillOverlap, rotation + 90);
}

void generateTriangleInfill(const Polygons& in_outline, int outlineOffset, Polygons& result,
                        int extrusionWidth, int lineSpacing, double infillOverlap,
                        double rotation)
{
    generateLineInfill(in_outline, outlineOffset, result, extrusionWidth, lineSpacing,
                       infillOverlap, rotation);
    generateLineInfill(in_outline, outlineOffset, result, extrusionWidth, lineSpacing,
                       infillOverlap, rotation + 60);
    generateLineInfill(in_outline, outlineOffset, result, extrusionWidth, lineSpacing,
                       infillOverlap, rotation + 120);
}

void addLineInfill(Polygons& result, const PointMatrix& matrix, const int scanline_min_idx, const int lineSpacing, const AABB boundary, std::vector<std::vector<int64_t>>& cutList, const int extrusionWidth)
{
    auto addLine = [&](Point from, Point to)
    {            
        PolygonRef p = result.newPoly();
        p.add(matrix.unapply(from));
        p.add(matrix.unapply(to));
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
    for(int64_t x = scanline_min_idx * lineSpacing; x < boundary.max.X; x += lineSpacing)
    {
        qsort(cutList[scanline_idx].data(), cutList[scanline_idx].size(), sizeof(int64_t), compare_int64_t);
        for(unsigned int i = 0; i + 1 < cutList[scanline_idx].size(); i += 2)
        {
            if (cutList[scanline_idx][i+1] - cutList[scanline_idx][i] < extrusionWidth / 5)
            {
                continue;
            }
            addLine(Point(x, cutList[scanline_idx][i]), Point(x, cutList[scanline_idx][i+1]));
        }
        scanline_idx += 1;
    }
}

void generateLineInfill(const Polygons& in_outline, int outlineOffset, Polygons& result, int extrusionWidth, int lineSpacing, double infillOverlap, const PointMatrix& rotation_matrix)
{
    NoZigZagConnectorProcessor lines_processor(rotation_matrix, result);
    generateLinearBasedInfill(in_outline, outlineOffset, result, extrusionWidth, lineSpacing, infillOverlap, rotation_matrix, lines_processor, false);
}


void generateZigZagInfill(const Polygons& in_outline, Polygons& result, const int extrusionWidth, const int lineSpacing, const double infillOverlap, const PointMatrix& rotation_matrix, const bool connect_zigzags, const bool use_endPieces)
{
    
    if (use_endPieces)
    {
        // return generateZigZagIninfill_endPieces(in_outline, result, extrusionWidth, lineSpacing, infillOverlap, rotation, connect_zigzags);
        if (connect_zigzags)
        {
            ZigzagConnectorProcessorConnectedEndPieces zigzag_processor(rotation_matrix, result);
            generateLinearBasedInfill(in_outline, 0, result, extrusionWidth, lineSpacing, infillOverlap, rotation_matrix, zigzag_processor, connect_zigzags);
        }
        else
        {
            ZigzagConnectorProcessorDisconnectedEndPieces zigzag_processor(rotation_matrix, result);
            generateLinearBasedInfill(in_outline, 0, result, extrusionWidth, lineSpacing, infillOverlap, rotation_matrix, zigzag_processor, connect_zigzags);
        }
    }
    else 
    {
        // return generateZigZagIninfill_noEndPieces(in_outline, result, extrusionWidth, lineSpacing, infillOverlap, rotation);
        ZigzagConnectorProcessorNoEndPieces zigzag_processor(rotation_matrix, result);
        generateLinearBasedInfill(in_outline, 0, result, extrusionWidth, lineSpacing, infillOverlap, rotation_matrix, zigzag_processor, connect_zigzags);
    }
}


void generateLinearBasedInfill(const Polygons& in_outline, const int outlineOffset, Polygons& result, const int extrusionWidth, const int lineSpacing, const double infillOverlap, const PointMatrix& rotation_matrix, ZigzagConnectorProcessor& zigzag_connector_processor, const bool connect_zigzags)
{
    if (lineSpacing == 0)
    {
        return;
    }
    if (in_outline.size() == 0)
    {
        return;
    }
    
    Polygons outline = ((outlineOffset)? in_outline.offset(outlineOffset) : in_outline).offset(extrusionWidth * infillOverlap / 100);
    if (outline.size() == 0)
    {
        return;
    }
    
    outline.applyMatrix(rotation_matrix);

    
    AABB boundary(outline);
    
    int scanline_min_idx = boundary.min.X / lineSpacing;
    int lineCount = (boundary.max.X + (lineSpacing - 1)) / lineSpacing - scanline_min_idx;
  
    std::vector<std::vector<int64_t> > cutList; // mapping from scanline to all intersections with polygon segments
    
    for(int scanline_idx = 0; scanline_idx < lineCount; scanline_idx++)
    {
        cutList.push_back(std::vector<int64_t>());
    }
    
    for(unsigned int poly_idx = 0; poly_idx < outline.size(); poly_idx++)
    {
        PolygonRef poly = outline[poly_idx];
        Point p0 = poly.back();
//         zigzag_connector_processor.registerPolyStart(p0); // TODO: remove this and the whole registerPolyStart function or uncomment and remove line below!!!!
        zigzag_connector_processor.registerVertex(p0);
        for(unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            Point p1 = poly[point_idx];
            if (p1.X == p0.X)
            {
                zigzag_connector_processor.registerVertex(p1);
                p0 = p1;
                continue; 
            }
            
            int scanline_idx0 = (p0.X + ((p0.X > 0)? -1 : -lineSpacing)) / lineSpacing; // -1 cause a linesegment on scanline x counts as belonging to scansegment x-1   ...
            int scanline_idx1 = (p1.X + ((p1.X > 0)? -1 : -lineSpacing)) / lineSpacing; // -linespacing because a line between scanline -n and -n-1 belongs to scansegment -n-1 (for n=positive natural number)
            int direction = 1;
            if (p0.X > p1.X) 
            { 
                direction = -1; 
                scanline_idx1 += 1; // only consider the scanlines in between the scansegments
            }
            else
            {
                scanline_idx0 += 1; // only consider the scanlines in between the scansegments
            }
            
            for(int scanline_idx = scanline_idx0; scanline_idx != scanline_idx1+direction; scanline_idx+=direction)
            {
                int x = scanline_idx * lineSpacing;
                int y = p1.Y + (p0.Y - p1.Y) * (x - p1.X) / (p0.X - p1.X);
                cutList[scanline_idx - scanline_min_idx].push_back(y);
                Point scanline_linesegment_intersection(x, y);
                zigzag_connector_processor.registerScanlineSegmentIntersection(scanline_linesegment_intersection, scanline_idx % 2 == 0);
            }
            zigzag_connector_processor.registerVertex(p1);
            p0 = p1;
        }
        zigzag_connector_processor.registerPolyFinished();
    }

    if (cutList.size() == 0)
    {
        return;
    }
    if (connect_zigzags && cutList.size() == 1 && cutList[0].size() <= 2)
    {
        return;  // don't add connection if boundary already contains whole outline!
    }

    addLineInfill(result, rotation_matrix, scanline_min_idx, lineSpacing, boundary, cutList, extrusionWidth);
}

}//namespace cura
