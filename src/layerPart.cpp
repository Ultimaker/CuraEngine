/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdio.h>

#include "layerPart.h"
#include "settings.h"

/*
The layer-part creation step is the first step in creating actual useful data for 3D printing.
It takes the result of the Slice step, which is an unordered list of polygons, and makes groups of polygons,
each of these groups is called a "part", which sometimes are also known as "islands". These parts represent
isolated areas in the 2D layer with possible holes.

Creating "parts" is an important step, as all elements in a single part should be printed before going to another part.
And all every bit inside a single part can be printed without the nozzle leaving the boundery of this part.

It's also the first step that stores the result in the "data storage" so all other steps can access it.
*/

namespace cura {

void createLayerWithParts(SliceLayer& storageLayer, SlicerLayer* layer, int unionAllType)
{
    storageLayer.openLines = layer->openPolygonList;

    if (unionAllType & FIX_HORRIBLE_UNION_ALL_TYPE_B)
    {
        for(unsigned int i=0; i<layer->polygonList.size(); i++)
        {
            if (layer->polygonList[i].orientation())
                layer->polygonList[i].reverse();
        }
    }
    
    vector<Polygons> result;
    if (unionAllType & FIX_HORRIBLE_UNION_ALL_TYPE_C)
        result = layer->polygonList.offset(1000).splitIntoParts(unionAllType);
    else
        result = layer->polygonList.splitIntoParts(unionAllType);
    for(unsigned int i=0; i<result.size(); i++)
    {
        storageLayer.parts.push_back(SliceLayerPart());
        if (unionAllType & FIX_HORRIBLE_UNION_ALL_TYPE_C)
        {
            storageLayer.parts[i].outline.add(result[i][0]);
            storageLayer.parts[i].outline = storageLayer.parts[i].outline.offset(-1000);
        }else
            storageLayer.parts[i].outline = result[i];
        storageLayer.parts[i].boundaryBox.calculate(storageLayer.parts[i].outline);
    }
}

void createLayerParts(SliceVolumeStorage& storage, Slicer* slicer, int unionAllType)
{
    for(unsigned int layerNr = 0; layerNr < slicer->layers.size(); layerNr++)
    {
        storage.layers.push_back(SliceLayer());
        storage.layers[layerNr].sliceZ = slicer->layers[layerNr].z;
        storage.layers[layerNr].printZ = slicer->layers[layerNr].z;
        createLayerWithParts(storage.layers[layerNr], &slicer->layers[layerNr], unionAllType);
    }
}

void dumpLayerparts(SliceDataStorage& storage, const char* filename)
{
    FILE* out = fopen(filename, "w");
    fprintf(out, "<!DOCTYPE html><html><body>");
    Point3 modelSize = storage.modelSize;
    Point3 modelMin = storage.modelMin;
    
    for(unsigned int volumeIdx=0; volumeIdx<storage.volumes.size(); volumeIdx++)
    {
        for(unsigned int layerNr=0;layerNr<storage.volumes[volumeIdx].layers.size(); layerNr++)
        {
            fprintf(out, "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" style=\"width: 500px; height:500px\">\n");
            SliceLayer* layer = &storage.volumes[volumeIdx].layers[layerNr];
            for(unsigned int i=0;i<layer->parts.size();i++)
            {
                SliceLayerPart* part = &layer->parts[i];
                for(unsigned int j=0;j<part->outline.size();j++)
                {
                    fprintf(out, "<polygon points=\"");
                    for(unsigned int k=0;k<part->outline[j].size();k++)
                        fprintf(out, "%f,%f ", float(part->outline[j][k].X - modelMin.x)/modelSize.x*500, float(part->outline[j][k].Y - modelMin.y)/modelSize.y*500);
                    if (j == 0)
                        fprintf(out, "\" style=\"fill:gray; stroke:black;stroke-width:1\" />\n");
                    else
                        fprintf(out, "\" style=\"fill:red; stroke:black;stroke-width:1\" />\n");
                }
            }
            fprintf(out, "</svg>\n");
        }
    }
    fprintf(out, "</body></html>");
    fclose(out);
}

}//namespace cura
