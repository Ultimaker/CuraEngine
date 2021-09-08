//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MESH_H
#define MESH_H

#include "settings/Settings.h"
#include "utils/AABB3D.h"
#include "utils/FMatrix4x3.h"
#include "utils/NoCopy.h"
#include <memory>

namespace cura
{
using mesh_idx_t = uint32_t; //!< Type for vertex and face indices

/*!
Vertex type to be used in a Mesh.

Keeps track of which faces connect to it.
*/
class MeshVertex
{
public:
    Point3 p; //!< location of the vertex
    std::vector<mesh_idx_t> connected_faces; //!< list of the indices of connected faces

    MeshVertex(Point3 p) : p(p) {connected_faces.reserve(8);} //!< doesn't set connected_faces
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
    mesh_idx_t vertex_index[3] = {}; //!< counter-clockwise ordering
    mesh_idx_t connected_face_index[3]; //!< same ordering as vertex_index (connected_face 0 is connected via vertex 0 and 1, etc.)
};

/*!
 * \brief 3D Spatial hashmap
 * Generates unique identifiers for deduplicated 3D vertices that are no closer to each other than a static distance (vertex_meld_distance).
 * Use Morton encoding and Python's dict probing.
 */
class HashMap3D
{
public:
    using Item = mesh_idx_t;
    using hash_t = uint64_t;

    HashMap3D() : HashMap3D(0){};
    HashMap3D(size_t nvertices);

    void clear(); //!< Drop content, free memory.

    /*!
     * Insert a new 3D point into the map and returns its identifier.
     * If a previously inserted point is found in a sphere of vertex_meld_distance radius, returns its identifier instead.
     * \param vertices Vector mapping identifiers to already inserted points. New identifier are generated from it's size. Read when rehashing.
     */
    Item insert(const Point3& p, const std::vector<MeshVertex>& vertices);

private:
    static constexpr uint8_t min_bits = 10; //!< Default log2 size of the map when zero initialized.
    std::unique_ptr<Item[]> map;
    hash_t mask = 0; //!< Bit mask for open adressing in map. (map.size()-1)
    mesh_idx_t free_vertices = 0; //!< Number of vertices that can be inserted before a rehash (when reaching 0)
    uint8_t bits = min_bits; //!< log2 size of map

    void init(); //!< Allocate the map. bits must be set beforehand.

    /*!
     * Inserts a point and its associated value in each of the 8 cubes overlapping
     * the sphere of radius=vertex_meld_distance centered on the point.
     * \param value Vertex identifier plus one (0 is the marker for empty slot)
     */
    void insert8(const Point3& p, Item value);

    /*!
     * Probing primitive.
     * Iterates over slots starting from the specified hash. Stops when an empty slot (=0) is found or when the predicates returns true.
     */
    template<typename P>
    Item& probe(hash_t hash, const P& predicate);
};


/*!
A Mesh is the most basic representation of a 3D model. It contains all the faces as MeshFaces.

See MeshFace for the specifics of how/when faces are connected.
*/
class Mesh : public NoCopy
{
    AABB3D aabb;
public:
    std::vector<MeshVertex> vertices; //!< list of all vertices in the mesh
    std::vector<MeshFace> faces; //!< list of all faces in the mesh
    Settings settings = {};
    std::string mesh_name;

    Mesh() = default;
    Mesh(size_t face_count);

    void addFace(Point3& v0, Point3& v1, Point3& v2); //!< add a face to the mesh without settings it's connected_faces.
    void clear(); //!< clears all data
    void finish(); //!< complete the model : set the connected_face_index fields of the faces.

    Point3 min() const; //!< min (in x,y and z) vertex of the bounding box
    Point3 max() const; //!< max (in x,y and z) vertex of the bounding box
    AABB3D getAABB() const; //!< Get the axis aligned bounding box
    void expandXY(int64_t offset); //!< Register applied horizontal expansion in the AABB

    /*!
     * Offset the whole mesh (all vertices and the bounding box).
     * \param offset The offset byu which to offset the whole mesh.
     */
    void offset(Point3 offset)
    {
        if (offset == Point3(0,0,0)) { return; }
        for(MeshVertex& v : vertices)
            v.p += offset;
        aabb.offset(offset);
    }

    /*!
     * Apply an affine transformation to this mesh's 3D data.
     * \param transformation The transformation to apply.
     */
    void transform(const FMatrix4x3& transformation);

    /*!
     * Gets whether this is a printable mesh (not an infill mesh, slicing mesh,
     * etc.)
     * \return True if it's a mesh that gets printed.
     */
    bool isPrinted() const;
private:
    HashMap3D spatial_map = {};

    mutable bool has_disconnected_faces = false; //!< Whether it has been logged that this mesh contains disconnected faces
    mutable bool has_overlapping_faces = false; //!< Whether it has been logged that this mesh contains overlapping faces
    mesh_idx_t findIndexOfVertex(const Point3& v); //!< find index of vertex close to the given point, or create a new vertex and return its index.

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
    ptrdiff_t getFaceIdxWithPoints(mesh_idx_t idx0, mesh_idx_t idx1, mesh_idx_t notFaceIdx, mesh_idx_t notFaceVertexIdx) const;
};

}//namespace cura
#endif // MESH_H
