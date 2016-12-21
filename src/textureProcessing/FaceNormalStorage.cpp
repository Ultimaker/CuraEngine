/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#include <limits> // numeric_limits
#include <cmath> // isnan

#include "FaceNormalStorage.h"

#include <math.h> // debug

#define NORMAL_LENGTH 10000

namespace cura
{

FaceNormalStorage::FaceNormalStorage(Mesh* mesh)
{
    face_normal_vertical_component.reserve(mesh->faces.size());
    for (MeshFace& face : mesh->faces)
    {
        Point3 p0 = mesh->vertices[face.vertex_index[0]].p;
        Point3 p1 = mesh->vertices[face.vertex_index[1]].p;
        Point3 p2 = mesh->vertices[face.vertex_index[2]].p;
        face_normal_vertical_component.emplace_back(computeFaceTanAngle(p0, p1, p2));
    }
}

float FaceNormalStorage::computeFaceTanAngle(const Point3 p0, const Point3 p1, const Point3 p2) const
{
    Point3 v01 = p1 - p0;
    Point3 v01_n = v01.normal(NORMAL_LENGTH);
    Point3 v02 = p2 - p0;
    Point3 v02_n = v02.normal(NORMAL_LENGTH);
    Point3 normal_dir = v01_n.cross(v02_n);
    coord_t z_component = normal_dir.z;
    coord_t xy_component = vSize(Point(normal_dir.x, normal_dir.y));
    if (xy_component > -2 && xy_component < 2)
    {
        if (z_component > 0)
        {
            return std::numeric_limits<float>::infinity();
        }
        else
        {
            return -1 * std::numeric_limits<float>::infinity();
        }
    }
    float ret = (float) z_component / (float) xy_component;
    assert(!std::isnan(ret));
    assert(!std::isnan(-ret));
    return ret;
}

float FaceNormalStorage::getFaceTanAngle(unsigned int face_idx)
{
    return face_normal_vertical_component[face_idx];
}


} // namespace cura