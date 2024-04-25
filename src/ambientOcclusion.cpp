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
 * Start by initializing the AO value for each face to zero.
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
    for (MeshFace& f : mesh_.faces_)
    {
        f.faceAO = 0;
    }

    size_t numRays = 10000;

    cura::parallel_for<size_t>(
        0,
        numRays,
        [&](size_t rayNumber)
        {
            std::pair<Point3LL, Point3D> pointPair = getRandomPointAndDirection();
            Point3LL startPoint = pointPair.first;
            Point3D direction = pointPair.second;
            doesRayIntersectMesh(startPoint, direction);
        });

    normalizeAmbientOcclusionValues();
}


// Get a random direction vector. This should generate a random point on a unit sphere.
std::pair<Point3LL, Point3D> AmbientOcclusion::getRandomPointAndDirection()
{
    // Choose two distinct faces randomly among 6 faces of the bounding box
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 5);

    size_t chosen_face_start = dis(gen);
    size_t chosen_face_end;
    do {
        chosen_face_end = dis(gen);
    } while(chosen_face_end == chosen_face_start);

    Point3LL startPoint;
    Point3LL endPoint;

    startPoint = getPoint(chosen_face_start, gen);

    // Switch for endPoint
    endPoint = getPoint(chosen_face_end, gen);
    // Compute direction vector from start to end points
    Point3D direction = endPoint - startPoint;
    direction = direction.normalized(); // Needed if your doesRayIntersectMesh method assumes a normalized direction vector
    return std::make_pair(startPoint, direction);
}

Point3LL AmbientOcclusion::getPoint(size_t chosen_face, std::mt19937 gen)
{
    Point3LL pointOnFace;
    std::uniform_real_distribution<> dis_f(0.0, 1.0);
    switch(chosen_face)
    {
    case 0:
        // Face x = min.x
        pointOnFace = Point3LL(mesh_.min().x_, dis_f(gen) * (mesh_.max().y_ - mesh_.min().y_) + mesh_.min().y_, dis_f(gen) * (mesh_.max().z_ - mesh_.min().z_) + mesh_.min().z_);
        break;
    case 1:
        // Face x = max.x
        pointOnFace = Point3LL(mesh_.max().x_, dis_f(gen) * (mesh_.max().y_ - mesh_.min().y_) + mesh_.min().y_, dis_f(gen) * (mesh_.max().z_ - mesh_.min().z_) + mesh_.min().z_);
        break;
    case 2:
        // Face y = min.y
        pointOnFace = Point3LL(dis_f(gen) * (mesh_.max().x_ - mesh_.min().x_) + mesh_.min().x_, mesh_.min().y_, dis_f(gen) * (mesh_.max().z_ - mesh_.min().z_) + mesh_.min().z_);
        break;
    case 3:
        // Face y = max.y
        pointOnFace = Point3LL(dis_f(gen) * (mesh_.max().x_ - mesh_.min().x_) + mesh_.min().x_, mesh_.max().y_, dis_f(gen) * (mesh_.max().z_ - mesh_.min().z_) + mesh_.min().z_);
        break;
    case 4:
        // Face z = min.z
        pointOnFace = Point3LL(dis_f(gen) * (mesh_.max().x_ - mesh_.min().x_) + mesh_.min().x_, dis_f(gen) * (mesh_.max().y_ - mesh_.min().y_) + mesh_.min().y_, mesh_.min().z_);
        break;
    case 5:
        // Face z = max.z
        pointOnFace = Point3LL(dis_f(gen) * (mesh_.max().x_ - mesh_.min().x_) + mesh_.min().x_, dis_f(gen) * (mesh_.max().y_ - mesh_.min().y_) + mesh_.min().y_, mesh_.max().z_);
        break;
    }
    return  pointOnFace;
}


// Check if a ray from point p in given direction intersects the mesh
void AmbientOcclusion::doesRayIntersectMesh(Point3LL p, Point3D direction)
{
    // An efficient approach to this would likely involve an acceleration structure like a KD-tree or BVH.
    // However, for simplicity, we can just interpolate through each face in the mesh and check for intersection
    for (auto& face : mesh_.faces_)
    {
        if (rayIntersectsTriangle(p, direction, face))
        {
            face.faceAO +=1;
        }
    }
}

// Normalize AO values
void AmbientOcclusion::normalizeAmbientOcclusionValues()
{
    // Get the maximal AO value in vertices
    float max_ao = 0;
    for (auto& face : mesh_.faces_)
    {
        if (face.faceAO > max_ao)
            max_ao = face.faceAO;
    }

    // Scale all AO values by maximal value
    for (auto& face : mesh_.faces_)
    {
        face.faceAO /= max_ao;
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