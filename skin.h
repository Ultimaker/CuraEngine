#ifndef SKIN_H
#define SKIN_H

void generateSkins(int layerNr, SliceVolumeStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount)
{
    SliceLayer* layer = &storage.layers[layerNr];

    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
    {
        SliceLayerPart* part = &layer->parts[partNr];
        Polygons temp;

        ClipperLib::OffsetPolygons(part->insets[part->insets.size() - 1], temp, -extrusionWidth/2, ClipperLib::jtSquare, 2, false);
        
        ClipperLib::Clipper downskinClipper;
        ClipperLib::Clipper upskinClipper;
        downskinClipper.AddPolygons(temp, ClipperLib::ptSubject);
        upskinClipper.AddPolygons(temp, ClipperLib::ptSubject);
        
        if (part->insets.size() > 1)
        {
            ClipperLib::Clipper thinWallClipper;

            ClipperLib::OffsetPolygons(part->insets[0], temp, -extrusionWidth / 2 - extrusionWidth * 15 / 100, ClipperLib::jtSquare, 2, false);
            thinWallClipper.AddPolygons(temp, ClipperLib::ptSubject);

            ClipperLib::OffsetPolygons(part->insets[1], temp, extrusionWidth * 6 / 10, ClipperLib::jtSquare, 2, false);
            thinWallClipper.AddPolygons(temp, ClipperLib::ptClip);

            thinWallClipper.Execute(ClipperLib::ctDifference, temp);
            downskinClipper.AddPolygons(temp, ClipperLib::ptSubject);
            upskinClipper.AddPolygons(temp, ClipperLib::ptSubject);
        }
        
        if (int(layerNr - downSkinCount) >= 0)
        {
            SliceLayer* layer2 = &storage.layers[layerNr - downSkinCount];
            for(unsigned int partNr2=0; partNr2<layer2->parts.size(); partNr2++)
            {
                if (part->boundaryBox.hit(layer2->parts[partNr2].boundaryBox))
                    downskinClipper.AddPolygons(layer2->parts[partNr2].insets[layer2->parts[partNr2].insets.size() - 1], ClipperLib::ptClip);
            }
        }
        if (int(layerNr + upSkinCount) < (int)storage.layers.size())
        {
            SliceLayer* layer2 = &storage.layers[layerNr + upSkinCount];
            for(unsigned int partNr2=0; partNr2<layer2->parts.size(); partNr2++)
            {
                if (part->boundaryBox.hit(layer2->parts[partNr2].boundaryBox))
                    upskinClipper.AddPolygons(layer2->parts[partNr2].insets[layer2->parts[partNr2].insets.size() - 1], ClipperLib::ptClip);
            }
        }
        
        ClipperLib::Polygons downSkin;
        ClipperLib::Polygons upSkin;
        downskinClipper.Execute(ClipperLib::ctDifference, downSkin);
        upskinClipper.Execute(ClipperLib::ctDifference, upSkin);
        
        {
            ClipperLib::Clipper skinCombineClipper;
            skinCombineClipper.AddPolygons(downSkin, ClipperLib::ptSubject);
            skinCombineClipper.AddPolygons(upSkin, ClipperLib::ptClip);
            skinCombineClipper.Execute(ClipperLib::ctUnion, part->skinOutline);
        }

        double minAreaSize = (2 * M_PI * (double(extrusionWidth) / 1000.0) * (double(extrusionWidth) / 1000.0)) * 0.3;
        for(unsigned int i=0; i<part->skinOutline.size(); i++)
        {
            double area = fabs(ClipperLib::Area(part->skinOutline[i])) / 1000.0 / 1000.0;
            if (area < minAreaSize) /* Only create an up/down skin if the area is large enough. So you do not create tiny blobs of "trying to fill" */
            {
                part->skinOutline.erase(part->skinOutline.begin() + i);
                i -= 1;
            }
        }
    }
}

void generateSparse(int layerNr, SliceVolumeStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount)
{
    SliceLayer* layer = &storage.layers[layerNr];

    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
    {
        SliceLayerPart* part = &layer->parts[partNr];

        Polygons temp;
        ClipperLib::OffsetPolygons(part->insets[part->insets.size() - 1], temp, -extrusionWidth/2, ClipperLib::jtSquare, 2, false);
        
        ClipperLib::Clipper downskinClipper;
        ClipperLib::Clipper upskinClipper;
        downskinClipper.AddPolygons(temp, ClipperLib::ptSubject);
        upskinClipper.AddPolygons(temp, ClipperLib::ptSubject);
        if (int(layerNr - downSkinCount) >= 0)
        {
            SliceLayer* layer2 = &storage.layers[layerNr - downSkinCount];
            for(unsigned int partNr2=0; partNr2<layer2->parts.size(); partNr2++)
            {
                if (layer2->parts[partNr2].insets.size() > 1)
                    if (part->boundaryBox.hit(layer2->parts[partNr2].boundaryBox))
                        downskinClipper.AddPolygons(layer2->parts[partNr2].insets[layer2->parts[partNr2].insets.size() - 2], ClipperLib::ptClip);
            }
        }
        if (int(layerNr + upSkinCount) < (int)storage.layers.size())
        {
            SliceLayer* layer2 = &storage.layers[layerNr + upSkinCount];
            for(unsigned int partNr2=0; partNr2<layer2->parts.size(); partNr2++)
            {
                if (layer2->parts[partNr2].insets.size() > 1)
                    if (part->boundaryBox.hit(layer2->parts[partNr2].boundaryBox))
                        upskinClipper.AddPolygons(layer2->parts[partNr2].insets[layer2->parts[partNr2].insets.size() - 2], ClipperLib::ptClip);
            }
        }
        
        ClipperLib::Polygons downSkin;
        ClipperLib::Polygons upSkin;
        ClipperLib::Polygons result;
        downskinClipper.Execute(ClipperLib::ctDifference, downSkin);
        upskinClipper.Execute(ClipperLib::ctDifference, upSkin);
        
        {
            ClipperLib::Clipper skinClipper;
            skinClipper.AddPolygons(downSkin, ClipperLib::ptSubject);
            skinClipper.AddPolygons(upSkin, ClipperLib::ptClip);
            skinClipper.Execute(ClipperLib::ctUnion, result);
        }

        double minAreaSize = 3.0;//(2 * M_PI * (double(config.extrusionWidth) / 1000.0) * (double(config.extrusionWidth) / 1000.0)) * 3;
        for(unsigned int i=0; i<result.size(); i++)
        {
            double area = fabs(ClipperLib::Area(result[i])) / 1000.0 / 1000.0;
            if (area < minAreaSize) /* Only create an up/down skin if the area is large enough. So you do not create tiny blobs of "trying to fill" */
            {
                result.erase(result.begin() + i);
                i -= 1;
            }
        }
        
        ClipperLib::Clipper sparseClipper;
        sparseClipper.AddPolygons(temp, ClipperLib::ptSubject);
        sparseClipper.AddPolygons(result, ClipperLib::ptClip);
        sparseClipper.Execute(ClipperLib::ctDifference, part->sparseOutline);
    }
}

#endif//SKIN_H
