#ifndef SKIRT_H
#define SKIRT_H

void generateSkirt(SliceDataStorage& storage, int distance, int extrusionWidth, int count)
{
    ClipperLib::Clipper skirtUnion;
    for(int skirtNr=0; skirtNr<count;skirtNr++)
    {
        for(unsigned int i=0; i<storage.layers[0].parts.size(); i++)
        {
            Polygons skirt;
            ClipperLib::OffsetPolygons(storage.layers[0].parts[i].outline, skirt, distance + extrusionWidth * skirtNr + extrusionWidth / 2, ClipperLib::jtSquare, 2, false);
            skirtUnion.AddPolygon(skirt[0], ClipperLib::ptSubject);
        }
    }
    skirtUnion.Execute(ClipperLib::ctUnion, storage.skirt);
}

#endif//SKIRT_H
