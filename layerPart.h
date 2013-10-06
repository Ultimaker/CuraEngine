/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef LAYERPART_H
#define LAYERPART_H

/*
The layer-part creation step is the first step in creating actual useful data for 3D printing.
It takes the result of the Slice step, which is an unordered list of polygons, and makes groups of polygons,
each of these groups is called a "part", which sometimes are also known as "islands". These parts represent
isolated areas in the 2D layer with possible holes.

Creating "parts" is an important step, as all elements in a single part should be printed before going to another part.
And all every bit inside a single part can be printed without the nozzle leaving the boundery of this part.

It's also the first step that stores the result in the "data storage" so all other steps can access it.
*/


//////////////////////////////////////////////////////////////////////////////////////////
//
// old API compatibility
// polytree to expolygons
// http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Types/ExPolygons.htm

struct ExPolygon
{
    ClipperLib::Polygon outer;
    ClipperLib::Polygons holes;
};
typedef std::vector<ExPolygon> ExPolygons;

void AddOuterPolyNodeToExPolygons(const ClipperLib::PolyNode * polynode, ExPolygons& expolygons)
{
    size_t cnt = expolygons.size();
    expolygons.resize(cnt + 1);
    expolygons[cnt].outer = polynode->Contour;
    expolygons[cnt].holes.resize(polynode->ChildCount());
    for (int i = 0; i < polynode->ChildCount(); ++i)
    {
        expolygons[cnt].holes[i] = polynode->Childs[i]->Contour;
        //Add outer polygons contained by (nested within) holes ...
        for (int j = 0; j < polynode->Childs[i]->ChildCount(); ++j)
            AddOuterPolyNodeToExPolygons(polynode->Childs[i]->Childs[j], expolygons);
    }
}

void PolyTreeToExPolygons(const ClipperLib::PolyTree * polytree, ExPolygons& expolygons)
{
    expolygons.clear();
    for (int i = 0; i < polytree->ChildCount(); ++i)
        AddOuterPolyNodeToExPolygons(polytree->Childs[i], expolygons);
}

//
//////////////////////////////////////////////////////////////////////////////////////////

void createLayerWithParts(SliceLayer& storageLayer, SlicerLayer* layer, int unionAllType)
{
    ClipperLib::Polygons polyList;
    for(unsigned int i=0; i<layer->polygonList.size(); i++)
    {
        ClipperLib::Polygon p;
        p.push_back(layer->polygonList[i][0]);
        for(unsigned int j=1; j<layer->polygonList[i].size(); j++)
        {
            p.push_back(layer->polygonList[i][j]);
        }
        if ((unionAllType & 0x02) && ClipperLib::Orientation(p))
            ClipperLib::ReversePolygon(p);
        polyList.push_back(p);
    }
    
    ExPolygons resultPolys;
    ClipperLib::PolyTree resultPolyTree;
    ClipperLib::Clipper clipper;
    clipper.AddPolygons(polyList, ClipperLib::ptSubject);
    if (unionAllType)
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    else
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree);
    
    PolyTreeToExPolygons(&resultPolyTree, resultPolys);
    
    for(unsigned int i=0; i<resultPolys.size(); i++)
    {
        storageLayer.parts.push_back(SliceLayerPart());
        
        storageLayer.parts[i].outline.push_back(resultPolys[i].outer);
        for(unsigned int j=0; j<resultPolys[i].holes.size(); j++)
        {
            storageLayer.parts[i].outline.push_back(resultPolys[i].holes[j]);
        }
        storageLayer.parts[i].boundaryBox.calculate(storageLayer.parts[i].outline);
    }
}

void createLayerParts(SliceVolumeStorage& storage, Slicer* slicer, int unionAllType)
{
    for(unsigned int layerNr = 0; layerNr < slicer->layers.size(); layerNr++)
    {
        storage.layers.push_back(SliceLayer());
        createLayerWithParts(storage.layers[layerNr], &slicer->layers[layerNr], unionAllType);
        //LayerPartsLayer(&slicer->layers[layerNr])
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

#endif//LAYERPART_H
