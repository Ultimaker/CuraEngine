#ifndef MULTIVOLUMES_H
#define MULTIVOLUMES_H

/* This file contains code to help fixing up and changing layers that are build from multiple volumes. */

void carveMultipleVolumes(vector<SliceVolumeStorage> &volumes)
{
    //Go trough all the volumes, and remove the previous volume outlines from our own outline, so we never have overlapped areas.
    for(unsigned int idx=0; idx < volumes.size(); idx++)
    {
        for(unsigned int idx2=0; idx2<idx; idx2++)
        {
            for(unsigned int layerNr=0; layerNr < volumes[idx].layers.size(); layerNr++)
            {
                SliceLayer* layer1 = &volumes[idx].layers[layerNr];
                SliceLayer* layer2 = &volumes[idx2].layers[layerNr];
                for(unsigned int p1 = 0; p1 < layer1->parts.size(); p1++)
                {
                    ClipperLib::Clipper clipper;
                    clipper.AddPolygons(layer1->parts[p1].outline, ClipperLib::ptSubject);
                    for(unsigned int p2 = 0; p2 < layer2->parts.size(); p2++)
                    {
                        clipper.AddPolygons(layer2->parts[p2].outline, ClipperLib::ptClip);
                    }
                    clipper.Execute(ClipperLib::ctDifference, layer1->parts[p1].outline);
                }
            }
        }
    }
}

//Expand each layer a bit and then keep the extra overlapping parts that overlap with other volumes.
//This generates some overlap in dual extrusion, for better bonding in touching parts.
void generateMultipleVolumesOverlap(vector<SliceVolumeStorage> &volumes, int overlap)
{
    if (volumes.size() < 2 || overlap <= 0) return;
    
    for(unsigned int layerNr=0; layerNr < volumes[0].layers.size(); layerNr++)
    {
        Polygons fullLayer;
        for(unsigned int volIdx = 0; volIdx < volumes.size(); volIdx++)
        {
            ClipperLib::Clipper fullLayerClipper;
            SliceLayer* layer1 = &volumes[volIdx].layers[layerNr];
            fullLayerClipper.AddPolygons(fullLayer, ClipperLib::ptSubject);
            for(unsigned int p1 = 0; p1 < layer1->parts.size(); p1++)
            {
                Polygons tmp;
                ClipperLib::OffsetPolygons(layer1->parts[p1].outline, tmp, 20, ClipperLib::jtSquare, 2, false);
                fullLayerClipper.AddPolygons(tmp, ClipperLib::ptClip);
            }
            fullLayerClipper.Execute(ClipperLib::ctUnion, fullLayer);
        }
        ClipperLib::OffsetPolygons(fullLayer, fullLayer, -20, ClipperLib::jtSquare, 2, false);
        
        for(unsigned int volIdx = 0; volIdx < volumes.size(); volIdx++)
        {
            SliceLayer* layer1 = &volumes[volIdx].layers[layerNr];
            for(unsigned int p1 = 0; p1 < layer1->parts.size(); p1++)
            {
                Polygons tmp;
                ClipperLib::OffsetPolygons(layer1->parts[p1].outline, tmp, overlap / 2, ClipperLib::jtSquare, 2, false);
                
                ClipperLib::Clipper clipper;
                clipper.AddPolygons(tmp, ClipperLib::ptClip);
                clipper.AddPolygons(fullLayer, ClipperLib::ptSubject);
                clipper.Execute(ClipperLib::ctIntersection, layer1->parts[p1].outline);
            }
        }
    }
}

#endif//MULTIVOLUMES_H
