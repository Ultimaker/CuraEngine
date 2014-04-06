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
                    for(unsigned int p2 = 0; p2 < layer2->parts.size(); p2++)
                    {
                        layer1->parts[p1].outline = layer1->parts[p1].outline.difference(layer2->parts[p2].outline);
                    }
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
            SliceLayer* layer1 = &volumes[volIdx].layers[layerNr];
            for(unsigned int p1 = 0; p1 < layer1->parts.size(); p1++)
            {
                fullLayer = fullLayer.unionPolygons(layer1->parts[p1].outline.offset(20));
            }
        }
        fullLayer = fullLayer.offset(-20);
        
        for(unsigned int volIdx = 0; volIdx < volumes.size(); volIdx++)
        {
            SliceLayer* layer1 = &volumes[volIdx].layers[layerNr];
            for(unsigned int p1 = 0; p1 < layer1->parts.size(); p1++)
            {
                layer1->parts[p1].outline = fullLayer.intersection(layer1->parts[p1].outline.offset(overlap / 2));
            }
        }
    }
}

#endif//MULTIVOLUMES_H
