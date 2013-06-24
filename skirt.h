/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SKIRT_H
#define SKIRT_H

void generateSkirt(SliceDataStorage& storage, int distance, int extrusionWidth, int count)
{
    for(int skirtNr=0; skirtNr<count;skirtNr++)
    {
        ClipperLib::Clipper skirtUnion;
        for(unsigned int volumeIdx = 0; volumeIdx < storage.volumes.size(); volumeIdx++)
        {
            if (storage.volumes[volumeIdx].layers.size() < 1) continue;
            SliceLayer* layer = &storage.volumes[volumeIdx].layers[0];
            for(unsigned int i=0; i<layer->parts.size(); i++)
            {
                Polygons skirt;
                ClipperLib::OffsetPolygons(layer->parts[i].outline, skirt, distance + extrusionWidth * skirtNr + extrusionWidth / 2, ClipperLib::jtSquare, 2, false);
                skirtUnion.AddPolygon(skirt[0], ClipperLib::ptSubject);
            }
        }
        Polygons skirtResult;
        skirtUnion.Execute(ClipperLib::ctUnion, skirtResult, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
        for(unsigned int n=0; n<skirtResult.size(); n++)
            storage.skirt.push_back(skirtResult[n]);
    }
}

#endif//SKIRT_H
