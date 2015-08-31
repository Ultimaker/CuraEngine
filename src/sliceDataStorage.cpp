#include "sliceDataStorage.h"

namespace cura
{


Polygons SliceDataStorage::getLayerOutlines(unsigned int layer_nr, bool include_helper_parts, bool external_polys_only)
{
    Polygons total;
    for (SliceMeshStorage& mesh : meshes)
    {
        SliceLayer& layer = mesh.layers[layer_nr];
        for (SliceLayerPart& part : layer.parts)
        {
            if (external_polys_only)
            {
                total.add(part.outline.outerPolygon());
            }
            else 
            {
                total.add(part.outline);
            }
        }
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






























} // namespace cura