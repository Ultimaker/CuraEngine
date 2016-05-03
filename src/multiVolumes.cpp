#include "multiVolumes.h"

namespace cura 
{
 
void carveMultipleVolumes(std::vector<Slicer*> &volumes)
{
    //Go trough all the volumes, and remove the previous volume outlines from our own outline, so we never have overlapped areas.
    for(unsigned int idx=0; idx < volumes.size(); idx++)
    {
        for(unsigned int idx2=0; idx2<idx; idx2++)
        {
            for(unsigned int layerNr=0; layerNr < volumes[idx]->layers.size(); layerNr++)
            {
                SlicerLayer& layer1 = volumes[idx]->layers[layerNr];
                SlicerLayer& layer2 = volumes[idx2]->layers[layerNr];
                layer1.polygons = layer1.polygons.difference(layer2.polygons);
            }
        }
    }
}
 
//Expand each layer a bit and then keep the extra overlapping parts that overlap with other volumes.
//This generates some overlap in dual extrusion, for better bonding in touching parts.
void generateMultipleVolumesOverlap(std::vector<Slicer*> &volumes, int overlap)
{
    if (volumes.size() < 2 || overlap <= 0) return;
    
    for(unsigned int layerNr=0; layerNr < volumes[0]->layers.size(); layerNr++)
    {
        Polygons fullLayer;
        for(unsigned int volIdx = 0; volIdx < volumes.size(); volIdx++)
        {
            SlicerLayer& layer1 = volumes[volIdx]->layers[layerNr];
            fullLayer = fullLayer.unionPolygons(layer1.polygons.offset(20)); // TODO: put hard coded value in a variable with an explanatory name (and make var a parameter, and perhaps even a setting?)
        }
        fullLayer = fullLayer.offset(-20); // TODO: put hard coded value in a variable with an explanatory name (and make var a parameter, and perhaps even a setting?)
        
        for(unsigned int volIdx = 0; volIdx < volumes.size(); volIdx++)
        {
            SlicerLayer& layer1 = volumes[volIdx]->layers[layerNr];
            layer1.polygons = fullLayer.intersection(layer1.polygons.offset(overlap / 2));
        }
    }
}
 

}//namespace cura
