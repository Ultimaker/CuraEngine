/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdio.h>

#include "utils/gettime.h"

#include "slicer.h"
#include "polygonOptimizer.h"

void SlicerLayer::makePolygons(OptimizedVolume* ov, bool keepNoneClosed, bool extensiveStitching)
{
    for(unsigned int startSegment=0; startSegment < segmentList.size(); startSegment++)
    {
        if (segmentList[startSegment].addedToPolygon)
            continue;
        
        ClipperLib::Polygon poly;
        poly.push_back(segmentList[startSegment].start);
        
        unsigned int segmentIndex = startSegment;
        bool canClose;
        while(true)
        {
            canClose = false;
            segmentList[segmentIndex].addedToPolygon = true;
            Point p0 = segmentList[segmentIndex].end;
            poly.push_back(p0);
            int nextIndex = -1;
            OptimizedFace* face = &ov->faces[segmentList[segmentIndex].faceIndex];
            for(unsigned int i=0;i<3;i++)
            {
                if (face->touching[i] > -1 && faceToSegmentIndex.find(face->touching[i]) != faceToSegmentIndex.end())
                {
                    Point p1 = segmentList[faceToSegmentIndex[face->touching[i]]].start;
                    Point diff = p0 - p1;
                    if (shorterThen(diff, 10))
                    {
                        if (faceToSegmentIndex[face->touching[i]] == (int)startSegment)
                            canClose = true;
                        if (segmentList[faceToSegmentIndex[face->touching[i]]].addedToPolygon)
                            continue;
                        nextIndex = faceToSegmentIndex[face->touching[i]];
                    }
                }
            }
            if (nextIndex == -1)
                break;
            segmentIndex = nextIndex;
        }
        if (canClose)
            polygonList.add(poly);
        else
            openPolygonList.add(poly);
    }
    //Clear the segmentList to save memory, it is no longer needed after this point.
    segmentList.clear();

    //Connecting polygons that are not closed yet, as models are not always perfect manifold we need to join some stuff up to get proper polygons
    //First link up polygon ends that are within 2 microns.
    for(unsigned int i=0;i<openPolygonList.size();i++)
    {
        if (openPolygonList[i].size() < 1) continue;
        for(unsigned int j=0;j<openPolygonList.size();j++)
        {
            if (openPolygonList[j].size() < 1) continue;
            
            Point diff = openPolygonList[i][openPolygonList[i].size()-1] - openPolygonList[j][0];
            int64_t distSquared = vSize2(diff);

            if (distSquared < 2 * 2)
            {
                if (i == j)
                {
                    polygonList.add(openPolygonList[i]);
                    openPolygonList[i].clear();
                    break;
                }else{
                    for(unsigned int n=0; n<openPolygonList[j].size(); n++)
                        openPolygonList[i].push_back(openPolygonList[j][n]);

                    openPolygonList[j].clear();
                }
            }
        }
    }
    
    //Next link up all the missing ends, closing up the smallest gaps first. This is an inefficient implementation which can run in O(n*n*n) time.
    while(1)
    {
        int64_t bestScore = 10000 * 10000;
        unsigned int bestA = -1;
        unsigned int bestB = -1;
        bool reversed = false;
        for(unsigned int i=0;i<openPolygonList.size();i++)
        {
            if (openPolygonList[i].size() < 1) continue;
            for(unsigned int j=0;j<openPolygonList.size();j++)
            {
                if (openPolygonList[j].size() < 1) continue;
                
                Point diff = openPolygonList[i][openPolygonList[i].size()-1] - openPolygonList[j][0];
                int64_t distSquared = vSize2(diff);
                if (distSquared < bestScore)
                {
                    bestScore = distSquared;
                    bestA = i;
                    bestB = j;
                    reversed = false;
                }

                if (i != j)
                {
                    Point diff = openPolygonList[i][openPolygonList[i].size()-1] - openPolygonList[j][openPolygonList[j].size()-1];
                    int64_t distSquared = vSize2(diff);
                    if (distSquared < bestScore)
                    {
                        bestScore = distSquared;
                        bestA = i;
                        bestB = j;
                        reversed = true;
                    }
                }
            }
        }
        
        if (bestScore >= 10000 * 10000)
            break;
        
        if (bestA == bestB)
        {
            polygonList.add(openPolygonList[bestA]);
            openPolygonList[bestA].clear();
        }else{
            if (reversed)
            {
                for(unsigned int n=openPolygonList[bestB].size()-1; int(n)>=0; n--)
                    openPolygonList[bestA].push_back(openPolygonList[bestB][n]);
            }else{
                for(unsigned int n=0; n<openPolygonList[bestB].size(); n++)
                    openPolygonList[bestA].push_back(openPolygonList[bestB][n]);
            }

            openPolygonList[bestB].clear();
        }
    }

    if (extensiveStitching)
    {
        //For extensive stitching find 2 open polygons that are touching 2 closed polygons.
        // Then find the sortest path over this polygon that can be used to connect the open polygons,
        // And generate a path over this shortest bit to link up the 2 open polygons.
        // (If these 2 open polygons are the same polygon, then the final result is a closed polyon)
        
        while(1)
        {
            unsigned int bestA = -1;
            unsigned int bestB = -1;
            gapCloserResult bestResult;
            bestResult.len = LLONG_MAX;
            bestResult.polygonIdx = -1;
            bestResult.pointIdxA = -1;
            bestResult.pointIdxB = -1;
            
            for(unsigned int i=0; i<openPolygonList.size(); i++)
            {
                if (openPolygonList[i].size() < 1) continue;
                
                {
                    gapCloserResult res = findPolygonGapCloser(openPolygonList[i][0], openPolygonList[i][openPolygonList[i].size()-1]);
                    if (res.len > 0 && res.len < bestResult.len)
                    {
                        bestA = i;
                        bestB = i;
                        bestResult = res;
                    }
                }

                for(unsigned int j=0; j<openPolygonList.size(); j++)
                {
                    if (openPolygonList[j].size() < 1 || i == j) continue;
                    
                    gapCloserResult res = findPolygonGapCloser(openPolygonList[i][0], openPolygonList[j][openPolygonList[j].size()-1]);
                    if (res.len > 0 && res.len < bestResult.len)
                    {
                        bestA = i;
                        bestB = j;
                        bestResult = res;
                    }
                }
            }
            
            if (bestResult.len < LLONG_MAX)
            {
                if (bestA == bestB)
                {
                    if (bestResult.pointIdxA == bestResult.pointIdxB)
                    {
                        polygonList.add(openPolygonList[bestA]);
                        openPolygonList[bestA].clear();
                    }
                    else if (bestResult.AtoB)
                    {
                        unsigned int n = polygonList.size();
                        polygonList.add(ClipperLib::Polygon());
                        for(unsigned int j = bestResult.pointIdxA; j != bestResult.pointIdxB; j = (j + 1) % polygonList[bestResult.polygonIdx].size())
                            polygonList[n].push_back(polygonList[bestResult.polygonIdx][j]);
                        for(unsigned int j = openPolygonList[bestA].size() - 1; int(j) >= 0; j--)
                            polygonList[n].push_back(openPolygonList[bestA][j]);
                        openPolygonList[bestA].clear();
                    }
                    else
                    {
                        unsigned int n = polygonList.size();
                        polygonList.add(openPolygonList[bestA]);
                        for(unsigned int j = bestResult.pointIdxB; j != bestResult.pointIdxA; j = (j + 1) % polygonList[bestResult.polygonIdx].size())
                            polygonList[n].push_back(polygonList[bestResult.polygonIdx][j]);
                        openPolygonList[bestA].clear();
                    }
                }
                else
                {
                    if (bestResult.pointIdxA == bestResult.pointIdxB)
                    {
                        for(unsigned int n=0; n<openPolygonList[bestA].size(); n++)
                            openPolygonList[bestB].push_back(openPolygonList[bestA][n]);
                        openPolygonList[bestA].clear();
                    }
                    else if (bestResult.AtoB)
                    {
                        ClipperLib::Polygon poly;
                        for(unsigned int n = bestResult.pointIdxA; n != bestResult.pointIdxB; n = (n + 1) % polygonList[bestResult.polygonIdx].size())
                            poly.push_back(polygonList[bestResult.polygonIdx][n]);
                        for(unsigned int n=poly.size()-1;int(n) >= 0; n--)
                            openPolygonList[bestB].push_back(poly[n]);
                        for(unsigned int n=0; n<openPolygonList[bestA].size(); n++)
                            openPolygonList[bestB].push_back(openPolygonList[bestA][n]);
                        openPolygonList[bestA].clear();
                    }
                    else
                    {
                        for(unsigned int n = bestResult.pointIdxB; n != bestResult.pointIdxA; n = (n + 1) % polygonList[bestResult.polygonIdx].size())
                            openPolygonList[bestB].push_back(polygonList[bestResult.polygonIdx][n]);
                        for(unsigned int n = openPolygonList[bestA].size() - 1; int(n) >= 0; n--)
                            openPolygonList[bestB].push_back(openPolygonList[bestA][n]);
                        openPolygonList[bestA].clear();
                    }
                }
            }
            else
            {
                break;
            }
        }
    }

    /*
    int q=0;
    for(unsigned int i=0;i<openPolygonList.size();i++)
    {
        if (openPolygonList[i].size() < 2) continue;
        if (!q) printf("***\n");
        printf("S: %f %f\n", float(openPolygonList[i][0].X), float(openPolygonList[i][0].Y));
        printf("E: %f %f\n", float(openPolygonList[i][openPolygonList[i].size()-1].X), float(openPolygonList[i][openPolygonList[i].size()-1].Y));
        q = 1;
    }
    */
    //if (q) exit(1);

    if (keepNoneClosed)
    {
        for(unsigned int n=0; n<openPolygonList.size(); n++)
        {
            if (openPolygonList[n].size() > 0)
                polygonList.add(openPolygonList[n]);
        }
    }
    //Clear the openPolygonList to save memory, the only reason to keep it after this is for debugging.
    //openPolygonList.clear();

    //Remove all the tiny polygons, or polygons that are not closed. As they do not contribute to the actual print.
    int snapDistance = 1000;
    for(unsigned int i=0;i<polygonList.size();i++)
    {
        int length = 0;
        
        for(unsigned int n=1; n<polygonList[i].size(); n++)
        {
            length += vSize(polygonList[i][n] - polygonList[i][n-1]);
            if (length > snapDistance)
                break;
        }
        if (length < snapDistance)
        {
            polygonList.remove(i);
            i--;
        }
    }

    //Finally optimize all the polygons. Every point removed saves time in the long run.
    optimizePolygons(polygonList);
}


Slicer::Slicer(OptimizedVolume* ov, int32_t initial, int32_t thickness, bool keepNoneClosed, bool extensiveStitching)
{
    modelSize = ov->model->modelSize;
    modelMin = ov->model->vMin;
    
    int layerCount = (modelSize.z - initial) / thickness + 1;
    fprintf(stderr, "Layer count: %i\n", layerCount);
    layers.resize(layerCount);
    
    for(unsigned int i=0; i<ov->faces.size(); i++)
    {
        Point3 p0 = ov->points[ov->faces[i].index[0]].p;
        Point3 p1 = ov->points[ov->faces[i].index[1]].p;
        Point3 p2 = ov->points[ov->faces[i].index[2]].p;
        int32_t minZ = p0.z;
        int32_t maxZ = p0.z;
        if (p1.z < minZ) minZ = p1.z;
        if (p2.z < minZ) minZ = p2.z;
        if (p1.z > maxZ) maxZ = p1.z;
        if (p2.z > maxZ) maxZ = p2.z;
        
        for(int32_t layerNr = (minZ - initial) / thickness; layerNr <= (maxZ - initial) / thickness; layerNr++)
        {
            int32_t z = layerNr * thickness + initial;
            if (z < minZ) continue;
            if (layerNr < 0) continue;
            
            SlicerSegment s;
            if (p0.z < z && p1.z >= z && p2.z >= z)
                s = project2D(p0, p2, p1, z);
            else if (p0.z > z && p1.z < z && p2.z < z)
                s = project2D(p0, p1, p2, z);

            else if (p1.z < z && p0.z >= z && p2.z >= z)
                s = project2D(p1, p0, p2, z);
            else if (p1.z > z && p0.z < z && p2.z < z)
                s = project2D(p1, p2, p0, z);

            else if (p2.z < z && p1.z >= z && p0.z >= z)
                s = project2D(p2, p1, p0, z);
            else if (p2.z > z && p1.z < z && p0.z < z)
                s = project2D(p2, p0, p1, z);
            else
            {
                //Not all cases create a segment, because a point of a face could create just a dot, and two touching faces
                //  on the slice would create two segments
                continue;
            }
            layers[layerNr].faceToSegmentIndex[i] = layers[layerNr].segmentList.size();
            s.faceIndex = i;
            s.addedToPolygon = false;
            layers[layerNr].segmentList.push_back(s);
        }
    }
    
    for(unsigned int layerNr=0; layerNr<layers.size(); layerNr++)
    {
        layers[layerNr].makePolygons(ov, keepNoneClosed, extensiveStitching);
    }
}

void Slicer::dumpSegmentsToHTML(const char* filename)
{
    float scale = std::max(modelSize.x, modelSize.y) / 1500;
    FILE* f = fopen(filename, "w");
    fprintf(f, "<!DOCTYPE html><html><body>\n");
    for(unsigned int i=0; i<layers.size(); i++)
    {
        fprintf(f, "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" style='width:%ipx;height:%ipx'>\n", int(modelSize.x / scale), int(modelSize.y / scale));
        fprintf(f, "<g fill-rule='evenodd' style=\"fill: gray; stroke:black;stroke-width:1\">\n");
        fprintf(f, "<path d=\"");
        for(unsigned int j=0; j<layers[i].polygonList.size(); j++)
        {
            ClipperLib::Polygon& p = layers[i].polygonList[j];
            for(unsigned int n=0; n<p.size(); n++)
            {
                if (n == 0)
                    fprintf(f, "M");
                else
                    fprintf(f, "L");
                fprintf(f, "%f,%f ", float(p[n].X - modelMin.x)/scale, float(p[n].Y - modelMin.y)/scale);
            }
            fprintf(f, "Z\n");
        }
        fprintf(f, "\"/>");
        fprintf(f, "</g>\n");
        for(unsigned int j=0; j<layers[i].openPolygonList.size(); j++)
        {
            ClipperLib::Polygon& p = layers[i].openPolygonList[j];
            if (p.size() < 1) continue;
            fprintf(f, "<polyline points=\"");
            for(unsigned int n=0; n<p.size(); n++)
            {
                fprintf(f, "%f,%f ", float(p[n].X - modelMin.x)/scale, float(p[n].Y - modelMin.y)/scale);
            }
            fprintf(f, "\" style=\"fill: none; stroke:red;stroke-width:1\" />\n");
        }
        fprintf(f, "</svg>\n");
    }
    fprintf(f, "</body></html>");
    fclose(f);
}
