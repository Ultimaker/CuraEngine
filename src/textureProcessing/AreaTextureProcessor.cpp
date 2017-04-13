/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */

#include <limits> // numeric_limits

#include "../utils/optional.h"

#include "AreaTextureProcessor.h"

namespace cura
{

Point AreaTextureProcessor::toPoint(const Point3& p)
{
    return Point(p.x, p.y);
}


AreaTextureProcessor::AreaTextureProcessor(const TexturedMesh& mesh, unsigned int layer_count)
: layer_height(mesh.getSettingInMicrons("layer_height"))
, initial_slice_z(mesh.getSettingInMicrons("layer_height_0") - layer_height / 2)
, layer_to_texture_faces(layer_count)
, face_normals(&mesh)
{
    const float min_tan_angle = 0.4 / 0.1 / sqrt(0.4 * 0.4 + 0.1 * 0.1);

    layer_to_texture_faces.resize(layer_count);

    for (unsigned int face_idx = 0; face_idx < mesh.faces.size(); face_idx++)
    {
        const MeshFace& face = mesh.faces[face_idx];
        const TexturedMesh::FaceTextureCoordIndices& face_texture = mesh.face_texture_indices[face_idx];
        float tan_angle = face_normals.getFaceTanAngle(face_idx);
        if (std::abs(tan_angle) < min_tan_angle)
        {
            continue;
        }
        TexturedFace textured_face;
        textured_face.verts[0].p = mesh.vertices[face.vertex_index[0]].p;
        textured_face.verts[0].uv = MatCoord(mesh.texture_coords[face_texture.index[0]], *mesh.material_base.getMat(face_texture.mat_id));
        textured_face.verts[1].p = mesh.vertices[face.vertex_index[1]].p;
        textured_face.verts[1].uv = MatCoord(mesh.texture_coords[face_texture.index[1]], *mesh.material_base.getMat(face_texture.mat_id));
        textured_face.verts[2].p = mesh.vertices[face.vertex_index[2]].p;
        textured_face.verts[2].uv = MatCoord(mesh.texture_coords[face_texture.index[2]], *mesh.material_base.getMat(face_texture.mat_id));

        registerTexturedFace(textured_face);
    }
}

void AreaTextureProcessor::registerTexturedFace(const AreaTextureProcessor::TexturedFace& face)
{
    AABB3D aabb3d;
    aabb3d.include(face.verts[0].p);
    aabb3d.include(face.verts[1].p);
    aabb3d.include(face.verts[2].p);

    AABB aabb(toPoint(aabb3d.min), toPoint(aabb3d.max));

    const int max = layer_to_texture_faces.size() - 1;
    const unsigned int min_layer = std::max(0, std::min(max, static_cast<int>((aabb3d.min.z - initial_slice_z) / layer_height)));
    const unsigned int max_layer = std::max(min_layer, std::min(max, static_cast<int>((aabb3d.max.z - initial_slice_z) / layer_height + 1)));
    for (unsigned int layer_nr = min_layer; layer_nr <= max_layer; layer_nr++)
    {
        RStarTree2D<TexturedFace>& layer_faces = layer_to_texture_faces[layer_nr];
        layer_faces.insert(face, aabb);
    }
}

std::optional<float> AreaTextureProcessor::getTextureColor(int layer_nr, Point location, ColourUsage color) const
{
    const RStarTree2D<TexturedFace>& layer_faces = layer_to_texture_faces[layer_nr];
    std::vector<AreaTextureProcessor::TexturedFace> faces = layer_faces.findEnclosing(location);

    coord_t layer_z = initial_slice_z + layer_nr * layer_height;

    std::optional<TexturedVertex> best;
    coord_t z_diff = std::numeric_limits<coord_t>::max();
    for (AreaTextureProcessor::TexturedFace& face : faces)
    {
        Polygon face_poly;
        face_poly.add(toPoint(face.verts[0].p));
        face_poly.add(toPoint(face.verts[1].p));
        face_poly.add(toPoint(face.verts[2].p));
        bool border_result = true;
        if (!face_poly.inside(location, border_result))
        {
            continue;
        }

        TexturedVertex mat_vert = getFaceUV(face, location);
        coord_t z_diff_here = std::abs(mat_vert.p.z - layer_z);
        if (!best || z_diff_here < z_diff)
        {
            z_diff = z_diff_here;
            best = mat_vert;
        }
    }
    if (best)
    {
        return std::optional<float>(best->uv.getColor(color));
    }
    else
    {
        return std::optional<float>();
    }
}

AreaTextureProcessor::TexturedVertex AreaTextureProcessor::getFaceUV(const AreaTextureProcessor::TexturedFace& face, Point location)
{
    // compute corresponding z
    // compute UV
    /*
     *    b
     *    |\
     *    | \
     *    |  \
     *    |   \
     *   y|....\ c
     *    |    /
     *   x|..l/
     *    |  /
     *    | /
     *    |/
     *    a
     */
    
    Point3 a3d = face.verts[0].p;
    Point3 b3d = face.verts[1].p;
    Point3 c3d = face.verts[2].p;
    FPoint a_t = face.verts[0].uv.coords;
    FPoint b_t = face.verts[1].uv.coords;
    FPoint c_t = face.verts[2].uv.coords;
    Point a = toPoint(a3d);
    Point b = toPoint(b3d);
    Point c = toPoint(c3d);
    Point ab = b - a;
    coord_t ab_length = vSize(ab);
    Point ac = c - a;
    coord_t ay_length = dot(ac, ab) / ab_length;
    Point3 y = a3d + (b3d - a3d) * ay_length / ab_length;
    FPoint y_tex = a_t + (b_t - a_t) * ay_length / ab_length;

    Point abT = turn90CCW(ab);
    coord_t cy_length = std::abs(dot(ac, abT) / ab_length);

    assert(std::abs(ay_length * ay_length + cy_length * cy_length - vSize2(ac)) < 50 && "ayc forms a right triangle, cause y is the projection of c onto ab.");
    
    Point al = location - a;
    coord_t ax_length = dot(al, ab) / ab_length;
    coord_t xl_length = std::abs(dot(al, abT) / ab_length);
    Point3 x = a3d + (b3d - a3d) * ax_length / ab_length;
    FPoint x_tex = a_t + (b_t - a_t) * ax_length / ab_length;
    Point3 l = x + (c3d - y) * xl_length / cy_length;
    FPoint l_lex = x_tex + (c_t - y_tex) * xl_length / cy_length;

#ifdef DEBUG
        Polygon face_poly;
        face_poly.add(toPoint(face.verts[0].p));
        face_poly.add(toPoint(face.verts[1].p));
        face_poly.add(toPoint(face.verts[2].p));
        bool border_result = true;
        assert(face_poly.inside(toPoint(l), border_result));
#endif // DEBUG

    return TexturedVertex{ l, MatCoord(l_lex, *face.verts[0].uv.mat) };
}



} // namespace cura