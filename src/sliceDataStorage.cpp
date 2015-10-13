#include "sliceDataStorage.h"

namespace cura
{

Polygons SliceLayer::getOutlines(bool external_polys_only)
{
    Polygons ret;
    getOutlines(ret, external_polys_only);
    return ret;
}

void SliceLayer::getOutlines(Polygons& result, bool external_polys_only)
{
    for (SliceLayerPart& part : parts)
    {
        if (external_polys_only)
        {
            result.add(part.outline.outerPolygon());
        }
        else 
        {
            result.add(part.outline);
        }
    }
}


Polygons SliceDataStorage::getLayerOutlines(unsigned int layer_nr, bool include_helper_parts, bool external_polys_only)
{
    if (layer_nr < 0)
    { // when processing raft
        if (include_helper_parts)
        {
            if (external_polys_only)
            {
                std::vector<PolygonsPart> parts = raftOutline.splitIntoParts();
                Polygons result;
                for (PolygonsPart& part : parts) 
                {
                    result.add(part.outerPolygon());
                }
                return result;
            }
            else 
            {
                return raftOutline;
            }
        }
        else 
        {
            return Polygons();
        }
    }
    else 
    {
        Polygons total;
        for (SliceMeshStorage& mesh : meshes)
        {
            SliceLayer& layer = mesh.layers[layer_nr];
            layer.getOutlines(total, external_polys_only);
            if (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL)
            {
                total = total.unionPolygons(layer.openPolyLines.offsetPolyLine(100));
            }
        }
        if (include_helper_parts)
        {
            if (support.generated) 
            {
                total.add(support.supportLayers[layer_nr].supportAreas);
                total.add(support.supportLayers[layer_nr].roofs);
            }
            total.add(primeTower.ground_poly);
        }
        return total;
    }
}






























} // namespace cura