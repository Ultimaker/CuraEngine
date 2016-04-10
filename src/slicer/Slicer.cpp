/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdio.h>

#include "../utils/gettime.h"
#include "../utils/logoutput.h"

#include "Slicer.h"

namespace cura {


SlicerSegment Slicer::project2D(unsigned int face_idx, Point3 p[3], unsigned int idx_shared, unsigned int idx_first, unsigned int idx_second, int32_t z) const
{
    Point3& p0 = p[idx_shared];
    Point3& p1 = p[idx_first];
    Point3& p2 = p[idx_second];
    SlicerSegment seg;
    seg.start.X = p0.x + int64_t(p1.x - p0.x) * int64_t(z - p0.z) / int64_t(p1.z - p0.z);
    seg.start.Y = p0.y + int64_t(p1.y - p0.y) * int64_t(z - p0.z) / int64_t(p1.z - p0.z);
    seg.end.X = p0.x + int64_t(p2.x - p0.x) * int64_t(z - p0.z) / int64_t(p2.z - p0.z);
    seg.end.Y = p0.y + int64_t(p2.y - p0.y) * int64_t(z - p0.z) / int64_t(p2.z - p0.z);
    mesh->registerFaceSlice(face_idx, idx_shared, idx_first, idx_second, z, seg.start, seg.end);
    return seg;
}


Slicer::Slicer(Mesh* mesh, int initial, int thickness, int layer_count, bool keep_none_closed, bool extensive_stitching)
: mesh(mesh)
, layer_height_0(initial)
, layer_height(thickness)
{
    assert(layer_count > 0);

    layers.resize(layer_count);
    
    for(int32_t layer_nr = 0; layer_nr < layer_count; layer_nr++)
    {
        layers[layer_nr].z = initial + thickness * layer_nr;
    }
    for (unsigned int face_idx = 0; face_idx < mesh->faces.size(); face_idx++)
    {
        MeshFace& face = mesh->faces[face_idx];
        Point3 p[3] = 
            { mesh->vertices[face.vertex_index[0]].p
            , mesh->vertices[face.vertex_index[1]].p
            , mesh->vertices[face.vertex_index[2]].p };
        Point3& p0 = p[0];
        Point3& p1 = p[1];
        Point3& p2 = p[2];
        int32_t minZ = p0.z;
        int32_t maxZ = p0.z;
        if (p1.z < minZ)
        {
            minZ = p1.z;
        }
        if (p2.z < minZ)
        {
            minZ = p2.z;
        }
        if (p1.z > maxZ)
        {
            maxZ = p1.z;
        }
        if (p2.z > maxZ)
        {
            maxZ = p2.z;
        }
        int32_t z = 0;
        for (int32_t layer_nr = (minZ - initial + thickness - 1) / thickness; z <= maxZ; layer_nr++) //  + thickness - 1 to get the first layer above or at minZ
        {
            SlicerSegment s;

            z = layer_nr * layer_height + layer_height_0;
            if (layer_nr < 0)
            {
                continue;
            }
            // below code checks the position of the points w.r.t. the layer z
            // also the direction of the resulting sliced line is determined
            // p0 is odd one out
            if (p0.z < z && p1.z >= z && p2.z >= z)
            {
                s = project2D(face_idx, p, 0, 2, 1, z);
            }
            else if (p0.z > z && p1.z < z && p2.z < z)
            {
                s = project2D(face_idx, p, 0, 1, 2, z);
            }
            // p1 is odd one out
            else if (p1.z < z && p0.z >= z && p2.z >= z)
            {
                s = project2D(face_idx, p, 1, 0, 2, z);
            }
            else if (p1.z > z && p0.z < z && p2.z < z)
            {
                s = project2D(face_idx, p, 1, 2, 0, z);
            }
            // p2 is odd one out
            else if (p2.z < z && p1.z >= z && p0.z >= z)
            {
                s = project2D(face_idx, p, 2, 1, 0, z);
            }
            else if (p2.z > z && p1.z < z && p0.z < z)
            {
                s = project2D(face_idx, p, 2, 0, 1, z);
            }
            else
            {
                //Not all cases create a segment, because a point of a face could create just a dot, and two touching faces
                //  on the slice would create two segments
                continue;
            }
            layers[layer_nr].face_idx_to_segment_index.insert(std::make_pair(face_idx, layers[layer_nr].segmentList.size()));
            s.faceIndex = face_idx;
            s.addedToPolygon = false;
            layers[layer_nr].segmentList.push_back(s);
        }
    }
    for (unsigned int layer_nr = 0; layer_nr < layers.size(); layer_nr++)
    {
        layers[layer_nr].makePolygons(mesh, keep_none_closed, extensive_stitching);
    }
}

}//namespace cura
