/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SLICER_H
#define SLICER_H

/*
    The Slicer creates layers of polygons from an optimized 3D model.
    The result of the Slicer is a list of polygons without any order or structure.
*/

class SlicerSegment
{
public:
    Point start, end;
    int faceIndex;
    bool addedToPolygon;
};

class SlicerLayer
{
public:
    std::vector<SlicerSegment> segmentList;
    std::map<int, int> faceToSegmentIndex;
    
    Polygons polygonList;
    Polygons openPolygonList;
    
    void makePolygons(OptimizedVolume* ov, bool keepNoneClosed)
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
                polygonList.push_back(poly);
            else
                openPolygonList.push_back(poly);
        }

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
                        polygonList.push_back(openPolygonList[i]);
                        openPolygonList.erase(openPolygonList.begin() + i);
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
                polygonList.push_back(openPolygonList[bestA]);
                openPolygonList.erase(openPolygonList.begin() + bestA);
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

        int q=0;
        for(unsigned int i=0;i<openPolygonList.size();i++)
        {
            if (openPolygonList[i].size() < 2) continue;
            if (!q) printf("***\n");
            printf("S: %f %f\n", float(openPolygonList[i][0].X), float(openPolygonList[i][0].Y));
            printf("E: %f %f\n", float(openPolygonList[i][openPolygonList[i].size()-1].X), float(openPolygonList[i][openPolygonList[i].size()-1].Y));
            q = 1;
        }
        //if (q) exit(1);

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
                polygonList.erase(polygonList.begin() + i);
                i--;
            }
        }
        
        if (keepNoneClosed)
        {
            while(openPolygonList.size() > 0)
            {
                polygonList.push_back(openPolygonList[0]);
                openPolygonList.erase(openPolygonList.begin());
            }
        }

        //Finally optimize all the polygons. Every point removed saves time in the long run.
        for(unsigned int i=0;i<polygonList.size();i++)
        {
            optimizePolygon(polygonList[i]);
        }
    }
};

class Slicer
{
public:
    std::vector<SlicerLayer> layers;
    Point3 modelSize, modelMin;
    
    Slicer(OptimizedVolume* ov, int32_t initial, int32_t thickness, bool keepNoneClosed)
    {
        modelSize = ov->model->modelSize;
        modelMin = ov->model->vMin;
        
        int layerCount = (modelSize.z - initial) / thickness + 1;
        fprintf(stdout, "Layer count: %i\n", layerCount);
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
        
        double t = getTime();
        int percDone;
        for(unsigned int layerNr=0; layerNr<layers.size(); layerNr++)
        {
            percDone = 100*layerNr/layers.size();
            if((getTime()-t)>2.0) fprintf(stdout, "\rProcessing layers... (%d percent)",percDone);
            layers[layerNr].makePolygons(ov, keepNoneClosed);
        }
        fprintf(stdout, "\rProcessed all layers in %5.1fs           \n",timeElapsed(t));
    }
        
    SlicerSegment project2D(Point3& p0, Point3& p1, Point3& p2, int32_t z)
    {
        SlicerSegment seg;
        seg.start.X = p0.x + int64_t(p1.x - p0.x) * int64_t(z - p0.z) / int64_t(p1.z - p0.z);
        seg.start.Y = p0.y + int64_t(p1.y - p0.y) * int64_t(z - p0.z) / int64_t(p1.z - p0.z);
        seg.end.X = p0.x + int64_t(p2.x - p0.x) * int64_t(z - p0.z) / int64_t(p2.z - p0.z);
        seg.end.Y = p0.y + int64_t(p2.y - p0.y) * int64_t(z - p0.z) / int64_t(p2.z - p0.z);
        return seg;
    }
    
    void dumpSegments(const char* filename)
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
};

#endif//SLICER_H
