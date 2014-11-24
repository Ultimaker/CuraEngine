#ifndef MESH_H
#define MESH_H

#include "settings.h"


class MeshVertex
{
public:
    Point3 p;
    std::vector<uint32_t> connected_faces;

    MeshVertex(Point3 p) : p(p) {}
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

/*! A Mesh is the most basic representation of a 3D model. It contains all the faces as SimpleTriangles. */
class Mesh : public SettingsBase
{
    //! The vertex_hash_map stores a index reference of each vertex for the hash of that location. Allows for quick retrieval of points with the same location.
    std::map<uint32_t, std::vector<uint32_t> > vertex_hash_map;
public:
    std::vector<MeshVertex> vertices;
    std::vector<MeshFace> faces;

    Mesh(SettingsBase* parent);

    void addFace(Point3& v0, Point3& v1, Point3& v2);
    void clear();
    void finish();

    Point3 min();
    Point3 max();

private:
    int findIndexOfVertex(Point3& v);
    int getFaceIdxWithPoints(int idx0, int idx1, int notFaceIdx);
};


#endif//MESH_H

