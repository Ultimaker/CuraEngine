/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdio.h>

#include "../utils/gettime.h"
#include "../utils/logoutput.h"
#include "../textureProcessing/MatCoord.h"

#include "Slicer.h"

namespace cura {


SlicerSegment Slicer::project2D(unsigned int face_idx, const Point3 p[3], unsigned int idx_shared, unsigned int idx_first, unsigned int idx_second, int32_t z, int32_t layer_nr)
{
    const Point3& p0 = p[idx_shared];
    const Point3& p1 = p[idx_first];
    const Point3& p2 = p[idx_second];

    SlicerSegment seg;

    seg.start.X = interpolate(z, p0.z, p1.z, p0.x, p1.x);
    seg.start.Y = interpolate(z, p0.z, p1.z, p0.y, p1.y);
    seg.end  .X = interpolate(z, p0.z, p2.z, p0.x, p2.x);
    seg.end  .Y = interpolate(z, p0.z, p2.z, p0.y, p2.y);
    if (textured_mesh)
    {
        MatSegment mat_segment;
        bool got_texture_coords = textured_mesh->sliceFaceTexture(face_idx, idx_shared, idx_first, idx_second, z, seg.start, seg.end, mat_segment);
        SlicerLayer& layer = layers[layer_nr];
        if (got_texture_coords && layer.texture_bump_map)
        {
            layer.texture_bump_map->registerTexturedFaceSlice(seg, mat_segment);
        }
    }
    return seg;
}

Slicer::Slicer(Mesh* mesh, int initial, int thickness, int slice_layer_count, bool keep_none_closed, bool extensive_stitching)
: mesh(mesh)
, textured_mesh(dynamic_cast<TexturedMesh*>(mesh))
{
    assert(slice_layer_count > 0);

    TimeKeeper slice_timer;

    std::optional<TextureBumpMapProcessor::Settings> bump_map_settings;
    if (mesh->getSettingBoolean("bump_map_enabled"))
    {
        bump_map_settings.emplace(mesh);
    }

    layers.resize(slice_layer_count, SlicerLayer(bump_map_settings));


    for(int32_t layer_nr = 0; layer_nr < slice_layer_count; layer_nr++)
    {
        layers[layer_nr].z = initial + thickness * layer_nr;
    }
    for(unsigned int face_idx = 0; face_idx < mesh->faces.size(); face_idx++)
    {
        const MeshFace& face = mesh->faces[face_idx];
        const MeshVertex& v0 = mesh->vertices[face.vertex_index[0]];
        const MeshVertex& v1 = mesh->vertices[face.vertex_index[1]];
        const MeshVertex& v2 = mesh->vertices[face.vertex_index[2]];
        Point3 p[3] =
            { mesh->vertices[face.vertex_index[0]].p
            , mesh->vertices[face.vertex_index[1]].p
            , mesh->vertices[face.vertex_index[2]].p };
        Point3& p0 = p[0];
        Point3& p1 = p[1];
        Point3& p2 = p[2];
        int32_t minZ = p0.z;
        int32_t maxZ = p0.z;
        if (p1.z < minZ) minZ = p1.z;
        if (p2.z < minZ) minZ = p2.z;
        if (p1.z > maxZ) maxZ = p1.z;
        if (p2.z > maxZ) maxZ = p2.z;
        int32_t layer_max = (maxZ - initial) / thickness;
        int32_t z = 0;
        for (int32_t layer_nr = (minZ - initial + thickness - 1) / thickness; layer_nr < layer_max; layer_nr++) //  + thickness - 1 to get the first layer above or at minZ
        {
            z = layer_nr * thickness + initial;
            if (z < minZ) continue;
            if (layer_nr < 0) continue;

            SlicerSegment s;
            s.endVertex = nullptr;
            int end_edge_idx = -1;
            if (p0.z < z && p1.z >= z && p2.z >= z)
            {
                s = project2D(face_idx, p, 0, 2, 1, z, layer_nr);
                end_edge_idx = 0;
                if (p1.z == z)
                {
                    s.endVertex = &v1;
                }
            }
            else if (p0.z > z && p1.z < z && p2.z < z)
            {
                s = project2D(face_idx, p, 0, 1, 2, z, layer_nr);
                end_edge_idx = 2;

            }

            else if (p1.z < z && p0.z >= z && p2.z >= z)
            {
                s = project2D(face_idx, p, 1, 0, 2, z, layer_nr);
                end_edge_idx = 1;
                if (p2.z == z)
                {
                    s.endVertex = &v2;
                }
            }
            else if (p1.z > z && p0.z < z && p2.z < z)
            {
                s = project2D(face_idx, p, 1, 2, 0, z, layer_nr);
                end_edge_idx = 0;

            }

            else if (p2.z < z && p1.z >= z && p0.z >= z)
            {
                s = project2D(face_idx, p, 2, 1, 0, z, layer_nr);
                end_edge_idx = 2;
                if (p0.z == z)
                {
                    s.endVertex = &v0;
                }
            }
            else if (p2.z > z && p1.z < z && p0.z < z)
            {
                s = project2D(face_idx, p, 2, 0, 1, z, layer_nr);
                end_edge_idx = 1;
            }
            else
            {
                //Not all cases create a segment, because a point of a face could create just a dot, and two touching faces
                //  on the slice would create two segments
                continue;
            }
            layers[layer_nr].face_idx_to_segment_idx.insert(std::make_pair(face_idx, layers[layer_nr].segments.size()));
            s.faceIndex = face_idx;
            s.endOtherFaceIdx = face.connected_face_index[end_edge_idx];
            s.addedToPolygon = false;
            layers[layer_nr].segments.push_back(s);
        }
    }
    log("slice of mesh took %.3f seconds\n",slice_timer.restart());
    for(unsigned int layer_nr=0; layer_nr<layers.size(); layer_nr++)
    {
        layers[layer_nr].makePolygons(mesh, keep_none_closed, extensive_stitching);
    }
    mesh->expandXY(mesh->getSettingInMicrons("xy_offset"));
    log("slice make polygons took %.3f seconds\n",slice_timer.restart());
}

}//namespace cura
