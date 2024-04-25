// Copyright (c) 2024 Ultimaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef AMBIENTOCCLUSION_H
#define AMBIENTOCCLUSION_H

#include "mesh.h"
#include "utils/AABB3D.h"
#include "utils/Matrix4x3D.h"


namespace cura
{

class AmbientOcclusion
{
public:
    AmbientOcclusion(Mesh& mesh)
        : mesh_(mesh)
    {
    }
    void calculate();

private:
    Mesh& mesh_;
    Point3D getRandomDirection();
    bool doesRayIntersectMesh(Point3LL p, Point3D direction);
    void normalizeAmbientOcclusionValues();
    bool rayIntersectsTriangle(Point3LL p, Point3D direction, MeshFace face);
    template<typename T, typename V>
    T crossProduct(const T& v1, const V& v2)
    {
        T cross_product;
        cross_product.x_ = v1.y_ * v2.z_ - v1.z_ * v2.y_;
        cross_product.y_ = v1.z_ * v2.x_ - v1.x_ * v2.z_;
        cross_product.z_ = v1.x_ * v2.y_ - v1.y_ * v2.x_;
        return cross_product;
    }
    template<typename T, typename V>
    float dotProduct(const T& v1, const V& v2)
    {
        return v1.x_ * v2.x_ + v1.y_ * v2.y_ + v1.z_ * v2.z_;
    }
};

} // namespace cura
#endif // AMBIENTOCCLUSION_H
