/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "support.h"

#include <cmath> // sqrt
#include <utility> // pair

namespace cura {

template<typename T> inline void swap(T& p0, T& p1)
{
    T tmp = p0;
    p0 = p1;
    p1 = tmp;
}

int cmp_SupportPoint(const void* a, const void* b)
{
    return ((SupportPoint*)a)->z - ((SupportPoint*)b)->z;
}

void generateSupportGrid(SupportStorage& storage, PrintObject* object, int supportAngle, bool supportEverywhere, int supportXYDistance, int supportZDistance)
{
    storage.areaSupport = false;
    storage.generated = false;
    if (supportAngle < 0)
        return;
    storage.generated = true;
    Point3 object_min = object->min();
    Point3 object_max = object->max();
    Point3 object_size = object_max - object_min;
    
    storage.gridOffset.X = object_min.x;
    storage.gridOffset.Y = object_min.y;
    storage.gridScale = 200;
    storage.gridWidth = (object_size.x / storage.gridScale) + 1;
    storage.gridHeight = (object_size.y / storage.gridScale) + 1;
    storage.grid = new std::vector<SupportPoint>[storage.gridWidth * storage.gridHeight];
    storage.angle = supportAngle;
    storage.everywhere = supportEverywhere;
    storage.XYDistance = supportXYDistance;
    storage.ZDistance = supportZDistance;

    for(Mesh& mesh : object->meshes)
    {
        for(MeshFace& face : mesh.faces)
        {
            Point3 v0 = mesh.vertices[face.vertex_index[0]].p;
            Point3 v1 = mesh.vertices[face.vertex_index[1]].p;
            Point3 v2 = mesh.vertices[face.vertex_index[2]].p;
            
            FPoint3 normal = FPoint3(v1 - v0).cross(FPoint3(v2 - v0));
            double normalSize = normal.vSize();
            
            double cosAngle = fabs(normal.z / normalSize);
            
            v0.x = (v0.x - storage.gridOffset.X) / storage.gridScale;
            v0.y = (v0.y - storage.gridOffset.Y) / storage.gridScale;
            v1.x = (v1.x - storage.gridOffset.X) / storage.gridScale;
            v1.y = (v1.y - storage.gridOffset.Y) / storage.gridScale;
            v2.x = (v2.x - storage.gridOffset.X) / storage.gridScale;
            v2.y = (v2.y - storage.gridOffset.Y) / storage.gridScale;

            if (v0.x > v1.x) swap(v0, v1);
            if (v1.x > v2.x) swap(v1, v2);
            if (v0.x > v1.x) swap(v0, v1);
            for(int64_t x=v0.x; x<v1.x; x++)
            {
                int64_t y0 = v0.y + (v1.y - v0.y) * (x - v0.x) / (v1.x - v0.x);
                int64_t y1 = v0.y + (v2.y - v0.y) * (x - v0.x) / (v2.x - v0.x);
                int64_t z0 = v0.z + (v1.z - v0.z) * (x - v0.x) / (v1.x - v0.x);
                int64_t z1 = v0.z + (v2.z - v0.z) * (x - v0.x) / (v2.x - v0.x);

                if (y0 > y1) { swap(y0, y1); swap(z0, z1); }
                for(int64_t y=y0; y<y1; y++)
                    storage.grid[x+y*storage.gridWidth].push_back(SupportPoint(z0 + (z1 - z0) * (y-y0) / (y1-y0), cosAngle));
            }
            for(int64_t x=v1.x; x<v2.x; x++)
            {
                int64_t y0 = v1.y + (v2.y - v1.y) * (x - v1.x) / (v2.x - v1.x);
                int64_t y1 = v0.y + (v2.y - v0.y) * (x - v0.x) / (v2.x - v0.x);
                int64_t z0 = v1.z + (v2.z - v1.z) * (x - v1.x) / (v2.x - v1.x);
                int64_t z1 = v0.z + (v2.z - v0.z) * (x - v0.x) / (v2.x - v0.x);

                if (y0 > y1) { swap(y0, y1); swap(z0, z1); }
                for(int64_t y=y0; y<y1; y++)
                    storage.grid[x+y*storage.gridWidth].push_back(SupportPoint(z0 + (z1 - z0) * (y-y0) / (y1-y0), cosAngle));
            }
        }
    }
    
    for(int32_t x=0; x<storage.gridWidth; x++)
    {
        for(int32_t y=0; y<storage.gridHeight; y++)
        {
            unsigned int n = x+y*storage.gridWidth;
            qsort(storage.grid[n].data(), storage.grid[n].size(), sizeof(SupportPoint), cmp_SupportPoint);
        }
    }
    storage.gridOffset.X += storage.gridScale / 2;
    storage.gridOffset.Y += storage.gridScale / 2;
}

bool SupportPolyGenerator::needSupportAt(Point p)
{
    if (p.X < 1) return false;
    if (p.Y < 1) return false;
    if (p.X >= storage.gridWidth - 1) return false;
    if (p.Y >= storage.gridHeight - 1) return false;
    if (done[p.X + p.Y * storage.gridWidth]) return false;
    
    unsigned int n = p.X+p.Y*storage.gridWidth;
    
    if (everywhere)
    {
        bool ok = false;
        for(unsigned int i=0; i<storage.grid[n].size(); i+=2)
        {
            if (storage.grid[n][i].cosAngle >= cosAngle && storage.grid[n][i].z - supportZDistance >= z && (i == 0 || storage.grid[n][i-1].z + supportZDistance < z))
            {
                ok = true;
                break;
            }
        }
        if (!ok) return false;
    }else{
        if (storage.grid[n].size() < 1) return false;
        if (storage.grid[n][0].cosAngle < cosAngle) return false;
        if (storage.grid[n][0].z - supportZDistance < z) return false;
    }
    return true;
}

void SupportPolyGenerator::lazyFill(Point startPoint)
{
    static int nr = 0;
    nr++;
    PolygonRef poly = polygons.newPoly();
    Polygon tmpPoly;

    while(1)
    {
        Point p = startPoint;
        done[p.X + p.Y * storage.gridWidth] = nr;
        while(needSupportAt(p + Point(1, 0)))
        {
            p.X ++;
            done[p.X + p.Y * storage.gridWidth] = nr;
        }
        tmpPoly.add(startPoint * storage.gridScale + storage.gridOffset - Point(storage.gridScale/2, 0));
        poly.add(p * storage.gridScale + storage.gridOffset);
        startPoint.Y++;
        while(!needSupportAt(startPoint) && startPoint.X <= p.X)
            startPoint.X ++;
        if (startPoint.X > p.X)
        {
            for(unsigned int n=0;n<tmpPoly.size();n++)
            {
                poly.add(tmpPoly[tmpPoly.size()-n-1]);
            }
            polygons.add(poly);
            return;
        }
        while(needSupportAt(startPoint - Point(1, 0)) && startPoint.X > 1)
            startPoint.X --;
    }
}
    
SupportPolyGenerator::SupportPolyGenerator(SupportStorage& storage, int32_t z, int layer_nr)
: storage(storage), z(z), everywhere(storage.everywhere)
{
    if (!storage.generated)
    {
        log("No support generated.\n");
        return;
    }
    if (storage.areaSupport)
    {
        polygons = storage.supportAreasPerLayer[layer_nr];
    } else
    {
        cosAngle = cos(double(90 - storage.angle) / 180.0 * M_PI) - 0.01;
        this->supportZDistance = storage.ZDistance;

        done = new int[storage.gridWidth*storage.gridHeight];
        memset(done, 0, sizeof(int) * storage.gridWidth*storage.gridHeight);
        
        for(int32_t y=1; y<storage.gridHeight; y++)
        {
            for(int32_t x=1; x<storage.gridWidth; x++)
            {
                if (!needSupportAt(Point(x, y)) || done[x + y * storage.gridWidth]) continue;
                
                lazyFill(Point(x, y));
            }
        }

        delete[] done;
        
        polygons = polygons.offset(storage.XYDistance);
    }
}




void generateSupportAreas(SliceDataStorage& storage, PrintObject* object, int layer_count)
{
    bool logStage = false; // whther to log at which stage of the support area generation we are (for debug)
    // given settings
    int supportAngle = object->getSettingInt("supportAngle");
    
    storage.support.generated = false;
    if (supportAngle < 0)
        return;
    
    bool supportEverywhere = object->getSettingInt("supportEverywhere") > 0;
    int supportXYDistance = object->getSettingInt("supportXYDistance");
    int supportZDistance = object->getSettingInt("supportZDistance");
    int supportZDistanceBottom = object->getSettingInt("supportZDistanceBottom");
    int supportZDistanceTop = object->getSettingInt("supportZDistanceTop");
    int supportJoinDistance = object->getSettingInt("supportJoinDistance");
    float backSupportBridge = static_cast<float>(object->getSettingInt("supportBridgeBack"))/100.0;
    int supportBottomStairDistance = object->getSettingInt("supportBottomStairDistance");
    int smoothing_distance = object->getSettingInt("supportAreaSmoothing"); 
    
    int supportTowerDiameter = object->getSettingInt("supportTowerDiameter");
    int supportMinAreaSqrt = object->getSettingInt("supportMinimalAreaSqrt");
    int supportTowerRoofAngle = object->getSettingInt("supportTowerRoofAngle");
    
    //std::cerr <<" towerDiameter=" << towerDiameter <<", supportMinAreaSqrt=" << supportMinAreaSqrt << std::endl;
    
    int min_smoothing_area = 100*100;
    int z_layer_distance_tower = 1;
        
    int layerThickness = object->getSettingInt("layerThickness");
    int extrusionWidth = object->getSettingInt("extrusionWidth"); // TODO check for layer0extrusionWidth!
    
    
    
    storage.support.angle = supportAngle;
    storage.support.everywhere = supportEverywhere;
    storage.support.XYDistance = supportXYDistance;
    storage.support.ZDistance = supportZDistance;
    storage.support.areaSupport = true;
    
    
    
    
    // derived settings:
    
    if (supportZDistanceBottom < 0) supportZDistanceBottom = supportZDistance;
    if (supportZDistanceTop < 0)    supportZDistanceTop = supportZDistance;
    
    
    int supportLayerThickness = layerThickness;
    
    int layerZdistanceTop       = supportZDistanceTop / supportLayerThickness + 1; // support must always be 1 layer below overhang
    int layerZdistanceBottom    = supportZDistanceBottom / supportLayerThickness; 

    double tanAngle = tan(double(storage.support.angle) / 180.0 * M_PI) - 0.01;
    int maxDistFromLowerLayer = tanAngle * supportLayerThickness; // max dist which can be bridged
    
    int expansionsDist = backSupportBridge * maxDistFromLowerLayer; // dist to expand overhang back toward model, effectively supporting stuff between the overhang and the perimeter of the lower layer
    
    int support_layer_count = layer_count;
    
    
    double tanTowerRoofAngle = tan(double(supportTowerRoofAngle) / 180.0 * M_PI);
    int towerRoofExpansionDistance = layerThickness / tanTowerRoofAngle;
    
    
    // computation
    
    if (logStage) log("joining model layers\n");
    
    // join model layers into polygons
    std::vector<Polygons> joinedLayers;
    std::vector<std::pair<int, std::vector<Polygons>>> overhang_points;
    for (int l = 0 ; l < layer_count ; l++)
    {
        joinedLayers.emplace_back();
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            SliceLayer& layer = mesh.layers[l];
            for (SliceLayerPart& part : layer.parts)
            {
                
                if (part.outline[0].area() < supportMinAreaSqrt * supportMinAreaSqrt) 
                //    && part.outline[0].area() > extrusionWidth * extrusionWidth * 4) // compensate for outline offset 
                {
                    Polygons part_poly = part.outline.offset(-extrusionWidth/2);
                    if (part_poly.size() > 0)
                    {
                        if (overhang_points.size() > 0 && overhang_points.back().first == l)
                            overhang_points.back().second.push_back(part_poly);
                        else 
                        {
                            std::vector<Polygons> small_part_polys;
                            small_part_polys.push_back(part_poly);
                            overhang_points.emplace_back<std::pair<int, std::vector<Polygons>>>(std::make_pair(l, small_part_polys));
                        }
                    }
                    
                }
                joinedLayers.back() = joinedLayers.back().unionPolygons(part.outline);
                
            }
        }
    }
        
    // initialization of supportAreasPerLayer
    for (int l = 0; l < layer_count ; l++)
        storage.support.supportAreasPerLayer.emplace_back();


    if (logStage) log("computing support");
    
    //auto roundToNearestDivisibleBy = [](int in, int divisor) { return in - in % divisor; };
    int overhang_points_pos = overhang_points.size() - 1;
    Polygons supportLayer_last;
    std::vector<Polygons> towerRoofs;
    for (int l = support_layer_count - 1 - layerZdistanceTop; l >= 0 ; l--)
    {
        
        
        // compute basic overhang and put in right layer ([layerZdistanceTOp] layers below)
        Polygons supportLayer_supportee =  joinedLayers[l+layerZdistanceTop];
        Polygons supportLayer_supported =  joinedLayers[l-1+layerZdistanceTop].offset(maxDistFromLowerLayer);
        Polygons basic_overhang = supportLayer_supportee.difference(supportLayer_supported);
        
        Polygons support_extension = basic_overhang.offset(expansionsDist);
        support_extension = support_extension.intersection(supportLayer_supported);
        support_extension = support_extension.intersection(supportLayer_supportee);
        
        Polygons overhang =  basic_overhang.unionPolygons(support_extension);
        
        Polygons& supportLayer_this = overhang; 
        
        { // handle towers
            // handle new tower roof tops
            int layer_overhang_point =  l + z_layer_distance_tower;
            if (overhang_points_pos >= 0 && layer_overhang_point < layer_count && 
                overhang_points[overhang_points_pos].first == layer_overhang_point) 
            {
                std::vector<Polygons>& overhang_points_here = overhang_points[overhang_points_pos].second;
                { // make sure we have the lowest point (make polys empty if they have small parts below)
                    if (overhang_points_pos > 0 && overhang_points[overhang_points_pos - 1].first == layer_overhang_point - 1)
                    {
                        std::vector<Polygons>& overhang_points_below = overhang_points[overhang_points_pos - 1].second;
                        for (Polygons& poly_here : overhang_points_here)
                            for (Polygons& poly_below : overhang_points_below)
                            {
                                poly_here = poly_here.difference(poly_below.offset(supportMinAreaSqrt*2));
                            }
                    }
                }
                for (Polygons& poly : overhang_points_here)
                    if (poly.size() > 0)
                        towerRoofs.push_back(poly);
                overhang_points_pos--;
            }
            
            // make tower roofs
            //for (Polygons& tower_roof : towerRoofs)
            for (int r = 0; r < towerRoofs.size(); r++)
            {
                supportLayer_this = supportLayer_this.unionPolygons(towerRoofs[r]);
                
                Polygons& tower_roof = towerRoofs[r];
                if (tower_roof[0].area() < supportTowerDiameter * supportTowerDiameter)
                {
                    towerRoofs[r] = tower_roof.offset(towerRoofExpansionDistance);
                }
            }
        }
        
        if (l+1 < support_layer_count)
        { // join with support from layer up
            Polygons& supportLayer_up = supportLayer_last;
            
            Polygons joined = supportLayer_this.unionPolygons(supportLayer_up);
            // join different parts
            if (supportJoinDistance > 0)
            {
                joined = joined.offset(supportJoinDistance);
                joined = joined.offset(-supportJoinDistance);
            }
            if (smoothing_distance > 0)
                joined = joined.smooth(smoothing_distance, min_smoothing_area);
        
            // remove layer
            Polygons insetted = joined.difference(joinedLayers[l]);
            supportLayer_this = insetted;                
            
        }
        
        
        supportLayer_last = supportLayer_this;
        
        // inset using X/Y distance
        if (supportLayer_this.size() > 0)
            supportLayer_this = supportLayer_this.difference(joinedLayers[l].offset(supportXYDistance));
        
        // move up from model
        if (layerZdistanceBottom > 0 && l >= layerZdistanceBottom)
        {
            int stepHeight = supportBottomStairDistance / supportLayerThickness + 1;
            int bottomLayer = ((l - layerZdistanceBottom) / stepHeight) * stepHeight;
            supportLayer_this = supportLayer_this.difference(joinedLayers[bottomLayer]);
        }
        
        storage.support.supportAreasPerLayer[l] = supportLayer_this;
        
    }
    
    // do stuff for when support on buildplate only
    if (!supportEverywhere)
    {
        if (logStage) log("supporting on buildplate only");
        Polygons touching_buildplate = storage.support.supportAreasPerLayer[0];
        for (int l = 1 ; l < storage.support.supportAreasPerLayer.size() ; l++)
        {
            Polygons& supportLayer = storage.support.supportAreasPerLayer[l];
            
            touching_buildplate = supportLayer.intersection(touching_buildplate);
            
            storage.support.supportAreasPerLayer[l] = touching_buildplate;
        }
    }

    
    joinedLayers.clear();
    if (logStage) log("finished area support");
    
    storage.support.generated = true;
}



/*!
 * adapted from generateLineInfill(.)
 * 
 * generate lines within the area of [in_outline], at regular intervals of [lineSpacing]
 * idea:
 * intersect a regular grid of 'scanlines' with the area inside [in_outline]
 * sigzag:
 * include pieces of boundary, connecting the lines, forming an accordion like zigzag instead of separate lines    |_|^|_|
 * 
 * we call the areas between two consecutive scanlines a 'scansegment'
 * 
 * algorithm:
 * 1. for each line segment of each polygon:
 *      store the intersections of that line segment with all scanlines in a mapping (vector of vectors) from scanline to intersections
 *      (zigzag): add boundary segments to result
 * 2. for each scanline:
 *      sort the associated intersections 
 *      and connect them using the even-odd rule
 * 
 * zigzag algorithm:
 * while walking around (each) polygon (1.)
 *  if polygon intersects with even scanline
 *      start boundary segment (add each following segment to the [result])
 *  when polygon intersects with a scanline again
 *      stop boundary segment (stop adding segments to the [result])
 *      if polygon intersects with even scanline again (instead of odd)
 *          dont add the last line segment to the boundary (unless [connect_zigzags])
 * 
 * 
 *     <--
 *     ___
 *    |   |   |
 *    |   |   |
 *    |   |___|
 *         -->
 * 
 *        ^ = even scanline
 * 
 * start boundary from even scanline! :D
 * 
 */
void generateZigZagSupport(const Polygons& in_outline, Polygons& result, int extrusionWidth, int lineSpacing, int infillOverlap, double rotation, bool connect_zigzags)
{    
//     if (in_outline.size() == 0) return;
//     Polygons outline = in_outline.offset(extrusionWidth * infillOverlap / 100);
    Polygons outline = in_outline; // .offset(extrusionWidth * infillOverlap / 100);
    if (outline.size() == 0) return;
    
    PointMatrix matrix(rotation);
    
    outline.applyMatrix(matrix);
    
    auto addLine = [&](Point from, Point to)
    {            
        PolygonRef p = result.newPoly();
        p.add(matrix.unapply(from));
        p.add(matrix.unapply(to));
    };   
        
    AABB boundary(outline);
    
    int scanline_min_idx = boundary.min.X / lineSpacing;
    int lineCount = (boundary.max.X + (lineSpacing - 1)) / lineSpacing - scanline_min_idx;
    
    std::vector<std::vector<int64_t> > cutList; // mapping from scanline to all intersections with polygon segments
    
    for(int n=0; n<lineCount; n++)
        cutList.push_back(std::vector<int64_t>());
    for(unsigned int polyNr=0; polyNr < outline.size(); polyNr++)
    {
        std::vector<Point> firstBoundarySegment;
        std::vector<Point> unevenBoundarySegment; // stored cause for connected_zigzags a boundary segment which ends in an uneven scanline needs to be included
        
        bool isFirstBoundarySegment = true;
        bool firstBoundarySegmentEndsInEven;
        
        bool isEvenScanSegment = false; 
        
        
        Point p0 = outline[polyNr][outline[polyNr].size()-1];
        Point lastPoint = p0;
        for(unsigned int i=0; i < outline[polyNr].size(); i++)
        {
            Point p1 = outline[polyNr][i];
            int64_t xMin = p1.X, xMax = p0.X;
            if (xMin == xMax) continue; 
            if (xMin > xMax) { xMin = p0.X; xMax = p1.X; }
            
            int scanline_idx0 = (p0.X + ((p0.X > 0)? -1 : -lineSpacing)) / lineSpacing; // -1 cause a linesegment on scanline x counts as belonging to scansegment x-1   ...
            int scanline_idx1 = (p1.X + ((p1.X > 0)? -1 : -lineSpacing)) / lineSpacing; // -linespacing because a line between scanline -n and -n-1 belongs to scansegment -n-1 (for n=positive natural number)
            int direction = 1;
            if (p0.X > p1.X) 
            { 
                direction = -1; 
                scanline_idx1 += 1; // only consider the scanlines in between the scansegments
            } else scanline_idx0 += 1; // only consider the scanlines in between the scansegments
            
            
            if (isFirstBoundarySegment) firstBoundarySegment.push_back(p0);
            for(int scanline_idx = scanline_idx0; scanline_idx != scanline_idx1+direction; scanline_idx+=direction)
            {
                int x = scanline_idx * lineSpacing;
                int y = p1.Y + (p0.Y - p1.Y) * (x - p1.X) / (p0.X - p1.X);
                cutList[scanline_idx - scanline_min_idx].push_back(y);
                
                
                bool last_isEvenScanSegment = isEvenScanSegment;
                if (scanline_idx % 2 == 0) isEvenScanSegment = true;
                else isEvenScanSegment = false;
                
                if (!isFirstBoundarySegment)
                {
                    if (last_isEvenScanSegment && (connect_zigzags || !isEvenScanSegment))
                        addLine(lastPoint, Point(x,y));
                    else if (connect_zigzags && !last_isEvenScanSegment && !isEvenScanSegment) // if we end an uneven boundary in an uneven segment
                    { // add whole unevenBoundarySegment (including the just obtained point)
                        for (int p = 1; p < unevenBoundarySegment.size(); p++)
                        {
                            addLine(unevenBoundarySegment[p-1], unevenBoundarySegment[p]);
                        }
                        addLine(unevenBoundarySegment[unevenBoundarySegment.size()-1], Point(x,y));
                        unevenBoundarySegment.clear();
                    } 
                    if (connect_zigzags && last_isEvenScanSegment && !isEvenScanSegment)
                        unevenBoundarySegment.push_back(Point(x,y));
                    else 
                        unevenBoundarySegment.clear();
                        
                }
                lastPoint = Point(x,y);
                
                if (isFirstBoundarySegment) 
                {
                    firstBoundarySegment.emplace_back(x,y);
                    firstBoundarySegmentEndsInEven = isEvenScanSegment;
                    isFirstBoundarySegment = false;
                }
                
            }
            if (!isFirstBoundarySegment)
            {
                if (isEvenScanSegment)
                    addLine(lastPoint, p1);
                else if (connect_zigzags)
                    unevenBoundarySegment.push_back(p1);
            }
            
            lastPoint = p1;
            p0 = p1;
        }
        
        if (isEvenScanSegment || isFirstBoundarySegment || connect_zigzags)
        {
            for (int i = 1; i < firstBoundarySegment.size() ; i++)
            {
                if (i < firstBoundarySegment.size() - 1 || !firstBoundarySegmentEndsInEven || connect_zigzags) // only add last element if connect_zigzags or boundary segment ends in uneven scanline
                    addLine(firstBoundarySegment[i-1], firstBoundarySegment[i]);
            }   
        }
        else if (!firstBoundarySegmentEndsInEven)
            addLine(firstBoundarySegment[firstBoundarySegment.size()-2], firstBoundarySegment[firstBoundarySegment.size()-1]);
    } 
    
    auto compare_int64_t = [](const void* a, const void* b)
    {
        int64_t n = (*(int64_t*)a) - (*(int64_t*)b);
        if (n < 0) return -1;
        if (n > 0) return 1;
        return 0;
    };
    
    int scanline_idx = 0;
    for(int64_t x = scanline_min_idx * lineSpacing; x < boundary.max.X; x += lineSpacing)
    {
        qsort(cutList[scanline_idx].data(), cutList[scanline_idx].size(), sizeof(int64_t), compare_int64_t);
        for(unsigned int i = 0; i + 1 < cutList[scanline_idx].size(); i+=2)
        {
            if (cutList[scanline_idx][i+1] - cutList[scanline_idx][i] < extrusionWidth / 5)
                continue;
            addLine(Point(x, cutList[scanline_idx][i]), Point(x, cutList[scanline_idx][i+1]));
        }
        scanline_idx += 1;
    }
}

}//namespace cura
