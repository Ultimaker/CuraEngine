/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "infill.h"

namespace cura {

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

void generateGridInfill(const Polygons& in_outline, Polygons& result,
                        int extrusionWidth, int lineSpacing, int infillOverlap,
                        double rotation)
{
    generateLineInfill(in_outline, result, extrusionWidth, lineSpacing * 2,
                       infillOverlap, rotation);
    generateLineInfill(in_outline, result, extrusionWidth, lineSpacing * 2,
                       infillOverlap, rotation + 90);
}

int compare_int64_t(const void* a, const void* b)
{
    int64_t n = (*(int64_t*)a) - (*(int64_t*)b);
    if (n < 0) return -1;
    if (n > 0) return 1;
    return 0;
}

/*!
 * generate lines within the area of [in_outline], at regular intervals of [lineSpacing]
 * idea:
 * intersect a regular grid of 'scanlines' with the area inside [in_outline]
 * sigzag:
 * include pieces of boundary, connecting the lines, forming an accordion like zigzag instead of separate lines    |_|^|_|
 * 
 * we call the areas between two consecutive scanlines a 'scansegment'
 * 
 * algorithm:
 * 1) for each line segment of each polygon:
 *      store the intersections of that line segment with all scanlines in a mapping (vector of vectors) from scanline to intersections
 *      (zigzag): add boundary segments to result
 * 2) for each scanline:
 *      sort the associated intersections 
 *      and connect them using the even-odd rule
 * 
 * zigzag algorithm:
 * while walking around (each) polygon
 *  if polygon intersects with even scanline
 *      start boundary segment (add each following segment to the [result])
 *  when polygon intersects with a scanline again
 *      stop boundary segment (stop adding segments to the [result])
 *      if polygon intersects with even scanline again (instead of odd)
 *          dont add the last line segment to the boundary
 */
void generateLineInfill(const Polygons& in_outline, Polygons& result, int extrusionWidth, int lineSpacing, int infillOverlap, double rotation)
{
    bool zigzag = true;

    
    Polygons outline = in_outline.offset(extrusionWidth * infillOverlap / 100);
    PointMatrix matrix(rotation);
    
    outline.applyMatrix(matrix);
    
    auto addLine = [&](Point from, Point to)
    {            
        PolygonRef p = result.newPoly();
        p.add(matrix.unapply(from));
        p.add(matrix.unapply(to));
    };
        
    AABB boundary(outline);
    
    boundary.min.X = ((boundary.min.X / lineSpacing) - 1) * lineSpacing;
    int lineCount = (boundary.max.X - boundary.min.X + (lineSpacing - 1)) / lineSpacing;
    
    std::vector<std::vector<int64_t> > cutList; // mapping from scanline to all intersections with polygon segments
    
    for(int n=0; n<lineCount; n++)
        cutList.push_back(std::vector<int64_t>());
    for(unsigned int polyNr=0; polyNr < outline.size(); polyNr++)
    {
        std::vector<Point> firstBoundarySegment;
        
        bool isFirstBoundarySegment = true;
        bool firstBoundarySegmentEndsInEven;
        
        bool isEvenScanSegment = false; 
        
        
        Point p0 = outline[polyNr][outline[polyNr].size()-1];
        Point lastPoint = p0;
        firstBoundarySegment.push_back(p0);
        for(unsigned int i=0; i < outline[polyNr].size(); i++)
        {
            Point p1 = outline[polyNr][i];
            int64_t xMin = p1.X, xMax = p0.X;
            if (xMin > xMax) { xMin = p0.X; xMax = p1.X; }
            
            int idx0 = (p0.X - boundary.min.X) / lineSpacing; // scanline index 0
            int idx1 = (p1.X - boundary.min.X) / lineSpacing; // scanline index 1
            int direction = 1;
            if (idx0 > idx1) { direction = -1; }
            
            if (isFirstBoundarySegment) firstBoundarySegment.push_back(p1);
            for(int idx = idx0; idx!=idx1+direction; idx+=direction)
            {
                int x = (idx * lineSpacing) + boundary.min.X + lineSpacing / 2;
                if (x < xMin) continue;
                if (x >= xMax) continue;
                int y = p1.Y + (p0.Y - p1.Y) * (x - p1.X) / (p0.X - p1.X);
                cutList[idx].push_back(y);
                
                
                bool last_isEvenScanSegment = isEvenScanSegment;
                if (idx % 2 == 0) isEvenScanSegment = true;
                else isEvenScanSegment = false;
                
                if (last_isEvenScanSegment && !isEvenScanSegment)
                    addLine(lastPoint, Point(x,y));
                lastPoint = Point(x,y);

                if (isFirstBoundarySegment) 
                {
                    firstBoundarySegment.push_back(lastPoint);
                    isFirstBoundarySegment = false;
                    firstBoundarySegmentEndsInEven = isEvenScanSegment;
                }
            }
            if (isEvenScanSegment)
                addLine(lastPoint, p1);
            lastPoint = p1;
            p0 = p1;
        }
        
        if (isEvenScanSegment)
        {
            for (int i = 1; i < firstBoundarySegment.size(); i++)
            {
                if (i < firstBoundarySegment.size() - 1 | !firstBoundarySegmentEndsInEven)
                    addLine(firstBoundarySegment[i-1], firstBoundarySegment[i]);
            }   
        }
    }
    
    int idx = 0;
    for(int64_t x = boundary.min.X + lineSpacing / 2; x < boundary.max.X; x += lineSpacing)
    {
        qsort(cutList[idx].data(), cutList[idx].size(), sizeof(int64_t), compare_int64_t);
        for(unsigned int i = 0; i + 1 < cutList[idx].size(); i+=2)
        {
            if (cutList[idx][i+1] - cutList[idx][i] < extrusionWidth / 5)
                continue;
            addLine(Point(x, cutList[idx][i]), Point(x, cutList[idx][i+1]));
        }
        idx += 1;
    }
}

}//namespace cura
