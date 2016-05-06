#ifndef MESH_H
#define MESH_H

#include "settings.h"
#include "utils/AABB3D.h"

namespace cura
{
/*!
Vertex type to be used in a Mesh.

Keeps track of which faces connect to it.
*/
class MeshVertex
{
public:
    Point3 p; //!< location of the vertex
    std::vector<uint32_t> connected_faces; //!< list of the indices of connected faces

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
    int vertex_index[3] = {-1}; //!< counter-clockwise ordering
    int connected_face_index[3]; //!< same ordering as vertex_index (connected_face 0 is connected via vertex 0 and 1, etc.)
};


/*!
A Mesh is the most basic representation of a 3D model. It contains all the faces as MeshFaces.

See MeshFace for the specifics of how/when faces are connected.
*/
class Mesh : public SettingsBase // inherits settings
{
    //! The vertex_hash_map stores a index reference of each vertex for the hash of that location. Allows for quick retrieval of points with the same location.
    std::unordered_map<uint32_t, std::vector<uint32_t> > vertex_hash_map;
    AABB3D aabb;
public:
    std::vector<MeshVertex> vertices;//!< list of all vertices in the mesh
    std::vector<MeshFace> faces; //!< list of all faces in the mesh

    Mesh(SettingsBaseVirtual* parent); //!< initializes the settings

    void addFace(Point3& v0, Point3& v1, Point3& v2); //!< add a face to the mesh without settings it's connected_faces.
    void clear(); //!< clears all data
    void finish(); //!< complete the model : set the connected_face_index fields of the faces.

    Point3 min() const; //!< min (in x,y and z) vertex of the bounding box
    Point3 max() const; //!< max (in x,y and z) vertex of the bounding box
    
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

private:
    int findIndexOfVertex(const Point3& v); //!< find index of vertex close to the given point, or create a new vertex and return its index.
    /*!
    Get the index of the face connected to the face with index \p notFaceIdx, via vertices \p idx0 and \p idx1.
    In case multiple faces connect with the same edge, return the next counter-clockwise face when viewing from \p idx1 to \p idx0.
    */
    int getFaceIdxWithPoints(int idx0, int idx1, int notFaceIdx) const;
};

}//namespace cura
#endif//MESH_H

