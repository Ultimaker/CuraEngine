#ifndef SKIRT_H
#define SKIRT_H

void generateSkirt(SliceDataStorage& storage, int distance, int extrusionWidth, int count)
{
    for(int skirtNr=0; skirtNr<count;skirtNr++)
    {
        ClipperLib::Clipper skirtUnion;
        for(unsigned int i=0; i<storage.layers[0].parts.size(); i++)
        {
            Polygons skirt;
            ClipperLib::OffsetPolygons(storage.layers[0].parts[i].outline, skirt, distance + extrusionWidth * skirtNr + extrusionWidth / 2, ClipperLib::jtSquare, 2, false);
            skirtUnion.AddPolygon(skirt[0], ClipperLib::ptSubject);
        }
        Polygons skirtResult;
        skirtUnion.Execute(ClipperLib::ctUnion, skirtResult, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
        for(unsigned int n=0; n<skirtResult.size(); n++)
            storage.skirt.push_back(skirtResult[n]);
    }
}

#endif//SKIRT_H
