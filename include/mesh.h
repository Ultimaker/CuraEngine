// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MESH_H
#define MESH_H

#include "settings/Settings.h"
#include "utils/AABB3D.h"
#include "utils/Matrix4x3D.h"

namespace cura
{
/*!
Vertex type to be used in a Mesh.

Keeps track of which faces connect to it.
*/
class MeshVertex
{
public:
    Point3LL p_; //!< location of the vertex
    std::vector<uint32_t> connected_faces_; //!< list of the indices of connected faces

    MeshVertex(Point3LL p)
        : p_(p)
    {
        connected_faces_.reserve(8);
    } //!< doesn't set connected_faces
};

/*! A MeshFace is a 3 dimensional model triangle with 3 points. These points are already converted to integers

A face has 3 connected faces, corresponding to its 3 edges.

Note that a correct model may have more than 2 faces connected via a single edge!
In such a case the face_index stored in connected_face_index is the one connected via the outside; see ASCII art below:

: horizontal slice through vertical edge connected to four faces :

\verbatim
[inside] x|
         x| <--+--- faces which contain each other in their connected_face_index fiels
   xxxxxxx|   \|/
   -------+-------
      ^   |xxxxxxx
      +-->|x
      |   |x [inside]
      |
    faces which contain each other in their connected_face_index fiels
\endverbatim
*/
class MeshFace
{
public:
    int vertex_index_[3] = { -1 }; //!< counter-clockwise ordering
    int connected_face_index_[3]; //!< same ordering as vertex_index (connected_face 0 is connected via vertex 0 and 1, etc.)
};


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

    Mesh(Settings& parent);
    Mesh();

    void addFace(Point3LL& v0, Point3LL& v1, Point3LL& v2); //!< add a face to the mesh without settings it's connected_faces.
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
     * Gets whether this is a printable mesh (not an infill mesh, slicing mesh,
     * etc.)
     * \return True if it's a mesh that gets printed.
     */
    bool isPrinted() const;

    /*!
     * Certain mesh types can interlock with each other. The interlock property provides
     * if this mesh can be used for interlocking with other meshes (for example support
     * blockers should not have an interlocking interface).
     *
     * \return True if an interface of the mesh could be interlocking with another mesh
     */
    bool canInterlock() const;

private:
    mutable bool has_disconnected_faces; //!< Whether it has been logged that this mesh contains disconnected faces
    mutable bool has_overlapping_faces; //!< Whether it has been logged that this mesh contains overlapping faces
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

} // namespace cura
#endif // MESH_H
