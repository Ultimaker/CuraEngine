// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "MeshFace.h"
#include "MeshVertex.h"
#include "TextureBitField.h"
#include "settings/Settings.h"
#include "utils/AABB3D.h"

namespace curaengine
{
class Image;
class Matrix4x3D;

/*!
A Mesh is the most basic representation of a 3D model. It contains all the faces as MeshFaces.

See MeshFace for the specifics of how/when faces are connected.
*/
class Mesh
{
    //! The vertex_hash_map stores a index reference of each vertex for the hash of that location. Allows for quick retrieval of points with the same location.
    std::unordered_map<uint32_t, std::vector<uint32_t>> vertex_hash_map_;
    AABB3D aabb_;

public:
    std::vector<MeshVertex> vertices_; //!< list of all vertices in the mesh
    std::vector<MeshFace> faces_; //!< list of all faces in the mesh
    Settings settings_;
    std::string mesh_name_;
    std::shared_ptr<Image> texture_;
    std::shared_ptr<TextureDataMapping> texture_data_mapping_;

    Mesh(const Settings& parent);
    Mesh(Settings&& parent);
    Mesh() = default;

    /*!
     *
     * @param v0 The 3D coordinates of vertex 0
     * @param v1 The 3D coordinates of vertex 1
     * @param v2 The 3D coordinates of vertex 2
     * @param uv0 The optional UV coordinates of vertex 0
     * @param uv1 The optional UV coordinates of vertex 1
     * @param uv2 The optional UV coordinates of vertex 2
     */
    void addFace(
        const Point3LL& v0,
        const Point3LL& v1,
        const Point3LL& v2,
        const std::optional<Point2F>& uv0 = std::nullopt,
        const std::optional<Point2F>& uv1 = std::nullopt,
        const std::optional<Point2F>& uv2 = std::nullopt); //!< add a face to the mesh without settings it's connected_faces.
    void clear(); //!< clears all data
    void finish(); //!< complete the model : set the connected_face_index fields of the faces.

    Point3LL min() const; //!< min (in x,y and z) vertex of the bounding box
    Point3LL max() const; //!< max (in x,y and z) vertex of the bounding box
    AABB3D getAABB() const; //!< Get the axis aligned bounding box
    void expandXY(int64_t offset); //!< Register applied horizontal expansion in the AABB

    /*!
     * Offset the whole mesh (all vertices and the bounding box).
     * \param offset The offset byu which to offset the whole mesh.
     */
    void translate(Point3LL offset)
    {
        if (offset == Point3LL(0, 0, 0))
        {
            return;
        }
        for (MeshVertex& v : vertices_)
            v.p_ += offset;
        aabb_.translate(offset);
    }

    /*!
     * Apply an affine transformation to this mesh's 3D data.
     * \param transformation The transformation to apply.
     */
    void transform(const Matrix4x3D& transformation);

    /*!
     * Gets whether this is a printable mesh (not a modifier mesh). This includes infill meshes, because they do get printed. Cutting meshes are not considered as printable though,
     * because they are early converted into regular meshes and should not be considered in subsequent algorithms.
     */
    bool isPrinted() const;

    /*! Gets whether this is a regular model mesh (not a modifier or infill mesh) */
    bool isModelMesh() const;

private:
    mutable bool has_disconnected_faces{ false }; //!< Whether it has been logged that this mesh contains disconnected faces
    mutable bool has_overlapping_faces{ false }; //!< Whether it has been logged that this mesh contains overlapping faces
    int findIndexOfVertex(const Point3LL& v); //!< find index of vertex close to the given point, or create a new vertex and return its index.

    /*!
     * Get the index of the face connected to the face with index \p notFaceIdx, via vertices \p idx0 and \p idx1.
     *
     * In case multiple faces connect with the same edge, return the next counter-clockwise face when viewing from \p idx1 to \p idx0.
     *
     * \param idx0 the first vertex index
     * \param idx1 the second vertex index
     * \param notFaceIdx the index of a face which shouldn't be returned
     * \param notFaceVertexIdx should be the third vertex of face \p notFaceIdx.
     * \return the face index of a face sharing the edge from \p idx0 to \p idx1
     */
    int getFaceIdxWithPoints(int idx0, int idx1, int notFaceIdx, int notFaceVertexIdx) const;
};

} // namespace curaengine
