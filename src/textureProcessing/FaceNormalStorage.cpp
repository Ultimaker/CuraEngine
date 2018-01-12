/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#include <limits> // numeric_limits
#include <cmath> // isnan

#include "FaceNormalStorage.h"

#include <math.h> // debug

// The length of a normalized vector. Cannot be 1 because points use integer logic.
#define NORMAL_LENGTH 10000

namespace cura
{

FaceNormalStorage::FaceNormalStorage(Mesh* mesh)
{
    face_normal.reserve(mesh->faces.size());
    for (MeshFace& face : mesh->faces)
    {
        Point3 p0 = mesh->vertices[face.vertex_index[0]].p;
        Point3 p1 = mesh->vertices[face.vertex_index[1]].p;
        Point3 p2 = mesh->vertices[face.vertex_index[2]].p;
        face_normal.emplace_back(computeFaceNormal(p0, p1, p2));
    }
}

Point3 FaceNormalStorage::computeFaceNormal(const Point3 p0, const Point3 p1, const Point3 p2) const
{
    Point3 v01 = p1 - p0;
    Point3 v01_n = v01.normal(NORMAL_LENGTH);
    Point3 v02 = p2 - p0;
    Point3 v02_n = v02.normal(NORMAL_LENGTH);
    Point3 normal_dir = v01_n.cross(v02_n);
    return normal_dir.normal(NORMAL_LENGTH);
}

float FaceNormalStorage::getFaceTanAngle(unsigned int face_idx)
{
    Point3 normal_dir = face_normal[face_idx];
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

float FaceNormalStorage::getFaceHorizontalComponent(unsigned int face_idx)
{
    Point3 normal_dir = face_normal[face_idx];
    return INT2MM(normal_dir.z) / INT2MM(NORMAL_LENGTH);
}

float FaceNormalStorage::getFaceVerticalComponent(unsigned int face_idx)
{
    Point3 normal_dir = face_normal[face_idx];
    coord_t xy_component = vSize(Point(normal_dir.x, normal_dir.y));
    return INT2MM(xy_component) / INT2MM(NORMAL_LENGTH);
}

} // namespace cura
