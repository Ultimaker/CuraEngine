#ifndef SKIN_H
#define SKIN_H

class SkinPart
{
public:
    ClipperLib::Polygons skin;
};

class SkinLayer
{
public:
    std::vector<SkinPart> parts;

    SkinLayer(int layerNr, std::vector<InsetLayer> &insetList, int downSkinCount, int upSkinCount)
    {
        InsetLayer* inset = &insetList[layerNr];

        for(unsigned int partNr=0; partNr<inset->parts.size(); partNr++)
        {
            InsetPart* part = &inset->parts[partNr];
            
            if (part->inset.size() < 1)
            {
                parts.push_back(SkinPart());
                continue;
            }
            
            ClipperLib::Clipper downskinClipper;
            ClipperLib::Clipper upskinClipper;
            downskinClipper.AddPolygons(part->inset[part->inset.size() - 1], ClipperLib::ptSubject);
            upskinClipper.AddPolygons(part->inset[part->inset.size() - 1], ClipperLib::ptSubject);
            if (int(layerNr - downSkinCount) >= 0)
            {
                InsetLayer* inset2 = &insetList[layerNr - downSkinCount];
                for(unsigned int partNr=0; partNr<inset2->parts.size(); partNr++)
                {
                    downskinClipper.AddPolygons(inset2->parts[partNr].inset[inset2->parts[partNr].inset.size() - 1], ClipperLib::ptClip);
                }
            }
            if (int(layerNr + upSkinCount) < (int)insetList.size())
            {
                InsetLayer* inset2 = &insetList[layerNr + upSkinCount];
                for(unsigned int partNr=0; partNr<inset2->parts.size(); partNr++)
                {
                    upskinClipper.AddPolygons(inset2->parts[partNr].inset[inset2->parts[partNr].inset.size() - 1], ClipperLib::ptClip);
                }
            }
            ClipperLib::Polygons downSkin;
            ClipperLib::Polygons upSkin;
            downskinClipper.Execute(ClipperLib::ctDifference, downSkin);
            upskinClipper.Execute(ClipperLib::ctDifference, upSkin);
            
            ClipperLib::Clipper skinClipper;
            skinClipper.AddPolygons(downSkin, ClipperLib::ptSubject);
            skinClipper.AddPolygons(upSkin, ClipperLib::ptClip);
            SkinPart newPart;
            skinClipper.Execute(ClipperLib::ctUnion, newPart.skin);
            parts.push_back(newPart);
        }
    }
};

#endif//SKIN_H
