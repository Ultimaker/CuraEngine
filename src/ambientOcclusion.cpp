// Copyright (c) 2024 Ultimaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ambientOcclusion.h"

#include "utils/Point3D.h"
#include "utils/ThreadPool.h"


namespace cura
{
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * Start by initializing the AO value for each vertex to zero.
For each vertex in the mesh:
Shoot a large number of rays (e.g., a few hundred) in random directions from the vertex. The higher the number of rays, the higher the quality of the AO approximation.
For each ray, detect if it intersects the geometry of the mesh. This involves performing intersection tests with each face in the mesh.
If the ray is obstructed, count it as an obstruction.
After all rays are tested, divide the number of obstructed rays by the total number of rays to get the AO for that vertex—ranges between 0(signalizing full occlusion) and 1(no
occlusion). Normalize AO values. Because these AO values are typically used as multipliers on color values, we usually want to normalize them to a 0.0 to 1.0 scale. This would
likely involve finding the maximum AO value and dividing all AO values by that maximum.
 */
void AmbientOcclusion::calculate()
{
    // Initialization of AO values
    for (MeshVertex& v : mesh_.vertices_)
    {
        v.vertexA0 = 0;
    }

    int numRays = 2000;

    for (MeshVertex& v : mesh_.vertices_)
    {
        int obstructions = 0;
        for (int ray = 0; ray < numRays; ray++)
        {
            // Cast a ray in a random direction
            Point3D direction = getRandomDirection();

            if (doesRayIntersectMesh(v.p_, direction))
            {
                obstructions++;
            }
        }
        v.vertexA0 = obstructions / float(numRays);
    }
    normalizeAmbientOcclusionValues();
}

// Get a random direction vector. This should generate a random point on a unit sphere.
Point3D AmbientOcclusion::getRandomDirection()
{
    // This can be achieved by generating random spherical coordinates and converting them to Cartesian coordinates.
    float theta = 2 * M_PI * (rand() / float(RAND_MAX)); // Azimuthal angle, [0, 2*pi]
    float phi = acos(2 * (rand() / float(RAND_MAX)) - 1); // Polar angle, [0, pi]

    // Conversion to Cartesian coordinates
    float x = sin(phi) * cos(theta);
    float y = sin(phi) * sin(theta);
    float z = cos(phi);

    return Point3D(x, y, z);
}

// Check if a ray from point p in given direction intersects the mesh
bool AmbientOcclusion::doesRayIntersectMesh(Point3LL p, Point3D direction)
{
    // An efficient approach to this would likely involve an acceleration structure like a KD-tree or BVH.
    // However, for simplicity, we can just interpolate through each face in the mesh and check for intersection
    for (auto& face : mesh_.faces_)
    {
        if (rayIntersectsTriangle(p, direction, face))
        {
            return true;
        }
    }
    return false;
}

// Normalize AO values
void AmbientOcclusion::normalizeAmbientOcclusionValues()
{
    // Get the maximal AO value in vertices
    float max_ao = 0;
    for (auto& v : mesh_.vertices_)
    {
        if (v.vertexA0 > max_ao)
            max_ao = v.vertexA0;
    }

    // Scale all AO values by maximal value
    for (auto& v : mesh_.vertices_)
    {
        v.vertexA0 /= max_ao;
    }
}

// Möller-Trumbore algorithm for ray-triangle intersection.
bool AmbientOcclusion::rayIntersectsTriangle(Point3LL p, Point3D direction, MeshFace face)
{
    Point3LL v0 = mesh_.vertices_[face.vertex_index_[0]].p_;
    Point3LL v1 = mesh_.vertices_[face.vertex_index_[1]].p_;
    Point3LL v2 = mesh_.vertices_[face.vertex_index_[2]].p_;

    // Compute the vectors along two edges of the triangle
    Point3LL edge1 = v1 - v0;
    Point3LL edge2 = v2 - v0;

    Point3D pvec = crossProduct<Point3D, Point3LL>(direction, edge2);
    float det = dotProduct<Point3D, Point3LL>(pvec, edge1);

    // If the determinant is near zero, the ray lies in the plane of the triangle
    if (fabs(det) < 0.000001)
        return false;

    float invDet = 1.0 / det;

    // Calculate distance from V0 to ray origin
    Point3LL tvec = p - v0;

    // Calculate U parameter and test bounds
    float u = dotProduct(pvec, tvec) * invDet;
    if (u < 0.0 || u > 1.0)
        return false;

    // Prepare to test V parameter
    Point3LL qvec = crossProduct<Point3LL, Point3LL>(tvec, edge1);

    // Calculate V parameter and test bounds
    float v = dotProduct<Point3D, Point3LL>(direction, qvec) * invDet;
    if (v < 0.0 || u + v > 1.0)
        return false;

    return true;
}
} // namespace cura
