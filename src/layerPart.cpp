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
And all every bit inside a single part can be printed without the nozzle leaving the boundary of this part.

It's also the first step that stores the result in the "data storage" so all other steps can access it.
*/

namespace cura {

void createLayerWithParts(SliceLayer& storageLayer, SlicerLayer* layer, bool union_layers, bool union_all_remove_holes)
{
    storageLayer.openLines = layer->openPolygons;

    if (union_all_remove_holes)
    {
        for(unsigned int i=0; i<layer->polygonList.size(); i++)
        {
            if (layer->polygonList[i].orientation())
                layer->polygonList[i].reverse();
        }
    }
    
    std::vector<PolygonsPart> result;
    result = layer->polygonList.splitIntoParts(union_layers || union_all_remove_holes);
    for(unsigned int i=0; i<result.size(); i++)
    {
        storageLayer.parts.push_back(SliceLayerPart());
        storageLayer.parts[i].outline = result[i];
        storageLayer.parts[i].boundaryBox.calculate(storageLayer.parts[i].outline);
    }
}

void createLayerParts(SliceMeshStorage& storage, Slicer* slicer, bool union_layers, bool union_all_remove_holes)
{
    for(unsigned int layer_nr = 0; layer_nr < slicer->layers.size(); layer_nr++)
    {
        storage.layers.push_back(SliceLayer());
        storage.layers[layer_nr].sliceZ = slicer->layers[layer_nr].z;
        storage.layers[layer_nr].printZ = slicer->layers[layer_nr].z;
        createLayerWithParts(storage.layers[layer_nr], &slicer->layers[layer_nr], union_layers, union_all_remove_holes);
        
        logProgress("layerparts", layer_nr + 1, slicer->layers.size());
    }
}

void dumpLayerparts(SliceDataStorage& storage, const char* filename)
{
    FILE* out = fopen(filename, "w");
    fprintf(out, "<!DOCTYPE html><html><body>");
    Point3 modelSize = storage.model_size;
    Point3 modelMin = storage.model_min;
    
    for(SliceMeshStorage& mesh : storage.meshes)
    {
        for(SliceLayer& layer : mesh.layers)
        {
            fprintf(out, "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" style=\"width: 500px; height:500px\">\n");
            for(SliceLayerPart& part : layer.parts)
            {
                for(unsigned int j=0;j<part.outline.size();j++)
                {
                    fprintf(out, "<polygon points=\"");
                    for(unsigned int k=0;k<part.outline[j].size();k++)
                        fprintf(out, "%f,%f ", float(part.outline[j][k].X - modelMin.x)/modelSize.x*500, float(part.outline[j][k].Y - modelMin.y)/modelSize.y*500);
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
