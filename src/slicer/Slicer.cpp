/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdio.h>

#include "../utils/gettime.h"
#include "../utils/logoutput.h"
#include "../textureProcessing/MatCoord.h"
#include "../textureProcessing/FaceNormalStorage.h"

#include "Slicer.h"

namespace cura {


void Slicer::project2D(unsigned int face_idx, const Point3 p[3], unsigned int idx_shared, unsigned int idx_first, unsigned int idx_second, int32_t z, int32_t layer_nr, SlicerSegment& seg)
{
    const Point3& p0 = p[idx_shared];
    const Point3& p1 = p[idx_first];
    const Point3& p2 = p[idx_second];

    seg.start.X = interpolate(z, p0.z, p1.z, p0.x, p1.x);
    seg.start.Y = interpolate(z, p0.z, p1.z, p0.y, p1.y);
    seg.end  .X = interpolate(z, p0.z, p2.z, p0.x, p2.x);
    seg.end  .Y = interpolate(z, p0.z, p2.z, p0.y, p2.y);
    if (textured_mesh)
    {
        MatSegment mat_segment;
        bool got_texture_coords = textured_mesh->sliceFaceTexture(face_idx, idx_shared, idx_first, idx_second, z, seg.start, seg.end, mat_segment);
        SlicerLayer& layer = layers[layer_nr];
        if (got_texture_coords)
        {
            if (layer.texture_bump_map)
            {
                layer.texture_bump_map->registerTexturedFaceSlice(seg, mat_segment);
            }
            if (texture_proximity_processor)
            {
                texture_proximity_processor->registerTexturedFaceSlice(seg, mat_segment, layer_nr);
            }
        }
    }
}

Slicer::Slicer(Mesh* mesh, int initial, int thickness, unsigned int slice_layer_count, bool keep_none_closed, bool extensive_stitching, TextureProximityProcessor* texture_proximity_processor)
: mesh(mesh)
, textured_mesh(dynamic_cast<TexturedMesh*>(mesh))
, texture_proximity_processor(texture_proximity_processor)
{
    assert((int) slice_layer_count > 0);

    TimeKeeper slice_timer;

    std::optional<TextureBumpMapProcessor::Settings> bump_map_settings;
    FaceNormalStorage* face_normal_storage = nullptr;
    if (mesh->getSettingBoolean("bump_map_enabled"))
    {
        bump_map_settings.emplace(mesh);
        if (mesh->getSettingAsRatio("bump_map_face_angle_correction") != 0.0)
        {
            face_normal_storage = new FaceNormalStorage(mesh);
        }
    }

    layers.reserve(slice_layer_count);
    for (uint32_t layer_nr = 0; layer_nr < slice_layer_count; layer_nr++)
    { // initialize all layers
        layers.emplace_back(layer_nr, mesh, bump_map_settings, face_normal_storage);
        assert(&layers.back() == &layers[layer_nr] && "We should just have emplaced the last layer!");
        layers[layer_nr].z = initial + thickness * layer_nr;
    }

    bool bump_map_alternate = mesh->getSettingBoolean("bump_map_alternate");
    int extruder_nr = mesh->getSettingAsIndex("extruder_nr");

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
        int32_t layer_min = (minZ - initial + thickness - 1) / thickness; //  + thickness - 1 to get the first layer above or at minZ
        for (int32_t layer_nr = layer_min; layer_nr <= layer_max; layer_nr++)
        {
            if (bump_map_alternate && layer_nr % 2 == extruder_nr) // TODO only works for the first two extruders!
            {
                continue;
            }
            int32_t z = layer_nr * thickness + initial;
            if (z < minZ) continue;
            if (layer_nr < 0) continue;

            SlicerSegment s;
            s.endVertex = nullptr;
            s.faceIndex = face_idx;
            assert(face_idx >= 0);
            s.addedToPolygon = false;
            if (p0.z < z && p1.z >= z && p2.z >= z)
            {
                s.endOtherFaceIdx = face.connected_face_index[0];
                if (p1.z == z)
                {
                    s.endVertex = &v1;
                }
                project2D(face_idx, p, 0, 2, 1, z, layer_nr, s);
            }
            else if (p0.z > z && p1.z < z && p2.z < z)
            {
                s.endOtherFaceIdx = face.connected_face_index[2];
                project2D(face_idx, p, 0, 1, 2, z, layer_nr, s);

            }

            else if (p1.z < z && p0.z >= z && p2.z >= z)
            {
                s.endOtherFaceIdx = face.connected_face_index[1];
                if (p2.z == z)
                {
                    s.endVertex = &v2;
                }
                project2D(face_idx, p, 1, 0, 2, z, layer_nr, s);
            }
            else if (p1.z > z && p0.z < z && p2.z < z)
            {
                s.endOtherFaceIdx = face.connected_face_index[0];
                project2D(face_idx, p, 1, 2, 0, z, layer_nr, s);

            }

            else if (p2.z < z && p1.z >= z && p0.z >= z)
            {
                s.endOtherFaceIdx = face.connected_face_index[2];
                if (p0.z == z)
                {
                    s.endVertex = &v0;
                }
                project2D(face_idx, p, 2, 1, 0, z, layer_nr, s);
            }
            else if (p2.z > z && p1.z < z && p0.z < z)
            {
                s.endOtherFaceIdx = face.connected_face_index[1];
                project2D(face_idx, p, 2, 0, 1, z, layer_nr, s);
            }
            else
            {
                //Not all cases create a segment, because a point of a face could create just a dot, and two touching faces
                //  on the slice would create two segments
                continue;
            }
            layers[layer_nr].face_idx_to_segment_idx.insert(std::make_pair(face_idx, layers[layer_nr].segments.size()));
            layers[layer_nr].segments.push_back(s);
        }
    }
    log("slice of mesh took %.3f seconds\n",slice_timer.restart());

    std::vector<SlicerLayer>& layers_ref = layers; // force layers not to be copied into the threads
#pragma omp parallel for default(none) shared(mesh,layers_ref) firstprivate(keep_none_closed, extensive_stitching)
    for(unsigned int layer_nr=0; layer_nr<layers_ref.size(); layer_nr++)
    {
        layers_ref[layer_nr].makePolygons(mesh, keep_none_closed, extensive_stitching);
    }

    mesh->expandXY(mesh->getSettingInMicrons("xy_offset"));
    log("slice make polygons took %.3f seconds\n",slice_timer.restart());

    if (face_normal_storage)
    {
        delete face_normal_storage;
    }
}

}//namespace cura
