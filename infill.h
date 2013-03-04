#ifndef INFILL_H
#define INFILL_H

void generateConcentricInfill(Polygons outline, Polygons& result, int offsets[], int offsetsSize)
{
    int step = 0;
    while(1)
    {
        ClipperLib::OffsetPolygons(outline, outline, -offsets[step], ClipperLib::jtSquare, 2, false);
        if (outline.size() < 1)
            break;
        step = (step + 1) % offsetsSize;
        for(unsigned int polygonNr=0; polygonNr<outline.size(); polygonNr++)
        {
            result.push_back(outline[polygonNr]);
        }
    }
}

int compare_int64_t(const void* a, const void* b)
{
    int64_t n = (*(int64_t*)a) - (*(int64_t*)b);
    if (n < 0) return -1;
    if (n > 0) return 1;
    return 0;
}

void generateLineInfill(Polygons outline, Polygons& result, int extrusionWidth, int lineSpacing, double rotation)
{
    ClipperLib::OffsetPolygons(outline, outline, -extrusionWidth * (0.5 - 0.15), ClipperLib::jtSquare, 2, false);
    PointMatrix matrix(rotation);
    PointMatrix unmatrix(-rotation);
    
    matrix.apply(outline);
    
    Point pMin = polymin(outline);
    Point pMax = polymax(outline);
    
    int lineCount = (pMax.X - pMin.X + (lineSpacing - 1)) / lineSpacing;
    vector<int64_t> cutList[lineCount];
    for(unsigned int polyNr=0; polyNr < outline.size(); polyNr++)
    {
        Point p1 = outline[polyNr][outline[polyNr].size()-1];
        for(unsigned int i=0; i < outline[polyNr].size(); i++)
        {
            Point p0 = outline[polyNr][i];
            int idx0 = (p0.X - pMin.X) / lineSpacing;
            int idx1 = (p1.X - pMin.X) / lineSpacing;
            int64_t xMin = p0.X, xMax = p1.X;
            if (p0.X > p1.X) { xMin = p1.X; xMax = p0.X; }
            if (idx0 > idx1) { int tmp = idx0; idx0 = idx1; idx1 = tmp; }
            for(int idx = idx0; idx<=idx1; idx++)
            {
                int x = (idx * lineSpacing) + pMin.X + lineSpacing / 2;
                if (x < xMin) continue;
                if (x >= xMax) continue;
                int y = p0.Y + (p1.Y - p0.Y) * (x - p0.X) / (p1.X - p0.X);
                cutList[idx].push_back(y);
            }
            p1 = p0;
        }
    }
    
    int idx = 0;
    for(int64_t x = pMin.X + lineSpacing / 2; x < pMax.X; x += lineSpacing)
    {
        qsort(cutList[idx].data(), cutList[idx].size(), sizeof(int64_t), compare_int64_t);
        for(unsigned int i = 0; i + 1 < cutList[idx].size(); i+=2)
        {
            if (cutList[idx][i+1] - cutList[idx][i] < 100) continue;
            ClipperLib::Polygon p;
            p.push_back(unmatrix.apply(Point(x, cutList[idx][i])));
            p.push_back(unmatrix.apply(Point(x, cutList[idx][i+1])));
            result.push_back(p);
        }
        idx += 1;
    }
}

#endif//INFILL_H
