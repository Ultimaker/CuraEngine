#ifndef LAYERPART_H
#define LAYERPART_H

class LayerPart
{
public:
    ClipperLib::Polygons polygons;
};

class LayerPartsLayer
{
public:
    std::vector<LayerPart> parts;
    
    LayerPartsLayer(SlicerLayer* layer)
    {
        ClipperLib::Polygons polyList;
        for(unsigned int i=0; i<layer->polygonList.size(); i++)
        {
            ClipperLib::Polygon p;
            p.push_back(layer->polygonList[i].points[0].IntPoint());
            unsigned int prev = 0;
            for(unsigned int j=1; j<layer->polygonList[i].points.size(); j++)
            {
                if ((layer->polygonList[i].points[j] - layer->polygonList[i].points[prev]).shorterThen(200))
                    continue;
                p.push_back(ClipperLib::IntPoint(layer->polygonList[i].points[j].x, layer->polygonList[i].points[j].y));
                prev = j;
            }
            polyList.push_back(p);
        }
        
        ClipperLib::ExPolygons resultPolys;
        ClipperLib::Clipper clipper;
        clipper.AddPolygons(polyList, ClipperLib::ptSubject);
        clipper.Execute(ClipperLib::ctUnion, resultPolys);
        
        for(unsigned int i=0; i<resultPolys.size(); i++)
        {
            LayerPart layerPart;
            layerPart.polygons.push_back(resultPolys[i].outer);
            for(unsigned int j=0; j<resultPolys[i].holes.size(); j++)
            {
                layerPart.polygons.push_back(resultPolys[i].holes[j]);
            }
            parts.push_back(layerPart);
        }
    }
    
    ~LayerPartsLayer()
    {
    }
    
    void dumpLayer(FILE* out, Point3 modelSize)
    {
        fprintf(out, "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" style=\"width: 150px; height:150px\">\n");
        for(unsigned int i=0;i<parts.size();i++)
        {
            for(unsigned int j=0;j<parts[i].polygons.size();j++)
            {
                fprintf(out, "<polygon points=\"");
                for(unsigned int k=0;k<parts[i].polygons[j].size();k++)
                {
                    fprintf(out, "%f,%f ", float(parts[i].polygons[j][k].X + modelSize.x/2)/modelSize.x*150, float(parts[i].polygons[j][k].Y + modelSize.y/2)/modelSize.y*150);
                }
                if (j == 0)
                    fprintf(out, "\" style=\"fill:gray; stroke:black;stroke-width:1\" />\n");
                else
                    fprintf(out, "\" style=\"fill:red; stroke:black;stroke-width:1\" />\n");
            }
        }
        fprintf(out, "</svg>\n");
    }
};

class LayerParts
{
public:
    std::vector<LayerPartsLayer> layers;
    Point3 modelSize;
    
    LayerParts(Slicer* slicer)
    {
        modelSize = slicer->modelSize;
        for(unsigned int layerNr = 0; layerNr < slicer->layers.size(); layerNr++)
        {
            layers.push_back(LayerPartsLayer(&slicer->layers[layerNr]));
        }
    }
    
    ~LayerParts()
    {
    }

    void dumpLayerparts(const char* filename)
    {
        FILE* out = fopen(filename, "w");
        fprintf(out, "<!DOCTYPE html><html><body>");
        
        for(unsigned int layerNr=0;layerNr<layers.size(); layerNr++)
        {
            layers[layerNr].dumpLayer(out, modelSize);
        }
        fprintf(out, "</body></html>");
        fclose(out);
    }
};

#endif//LAYERPART_H
