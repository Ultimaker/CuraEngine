#include "mesh.h"
#include "utils/logoutput.h"


const int vertex_meld_distance = MM2INT(0.03);
static inline uint32_t pointHash(Point3& p)
{
    return ((p.x + vertex_meld_distance/2) / vertex_meld_distance) ^ (((p.y + vertex_meld_distance/2) / vertex_meld_distance) << 10) ^ (((p.z + vertex_meld_distance/2) / vertex_meld_distance) << 20);
}

Mesh::Mesh(SettingsBase* parent)
: SettingsBase(parent)
{
}

void Mesh::addFace(Point3& v0, Point3& v1, Point3& v2)
{
    int vi0 = findIndexOfVertex(v0);
    int vi1 = findIndexOfVertex(v1);
    int vi2 = findIndexOfVertex(v2);
    if (vi0 == vi1 || vi1 == vi2 || vi0 == vi2) return; // the face has two vertices which get assigned the same location. Don't add the face.

    int idx = faces.size(); // index of face to be added
    faces.emplace_back();
    MeshFace& face = faces[idx];
    face.vertex_index[0] = vi0;
    face.vertex_index[1] = vi1;
    face.vertex_index[2] = vi2;
    vertices[face.vertex_index[0]].connected_faces.push_back(idx);
    vertices[face.vertex_index[1]].connected_faces.push_back(idx);
    vertices[face.vertex_index[2]].connected_faces.push_back(idx);
}

void Mesh::clear()
{
    faces.clear();
    vertices.clear();
    vertex_hash_map.clear();
}

void Mesh::finish()
{
    // Finish up the mesh, clear the vertex_hash_map, as it's no longer needed from this point on and uses quite a bit of memory.
    vertex_hash_map.clear();

    // For each face, store which other face is connected with it.
    for(unsigned int i=0; i<faces.size(); i++)
    {
        MeshFace& face = faces[i];
        face.connected_face_index[0] = getFaceIdxWithPoints(face.vertex_index[0], face.vertex_index[1], i); // faces are connected via the outside
        face.connected_face_index[1] = getFaceIdxWithPoints(face.vertex_index[1], face.vertex_index[2], i);
        face.connected_face_index[2] = getFaceIdxWithPoints(face.vertex_index[2], face.vertex_index[0], i);
    }
}

Point3 Mesh::min()
{
    if (vertices.size() < 1)
        return Point3(0, 0, 0);
    Point3 ret = vertices[0].p;
    for(unsigned int i=0; i<vertices.size(); i++)
    {
        ret.x = std::min(ret.x, vertices[i].p.x);
        ret.y = std::min(ret.y, vertices[i].p.y);
        ret.z = std::min(ret.z, vertices[i].p.z);
    }
    return ret;
}
Point3 Mesh::max()
{
    if (vertices.size() < 1)
        return Point3(0, 0, 0);
    Point3 ret = vertices[0].p;
    for(unsigned int i=0; i<vertices.size(); i++)
    {
        ret.x = std::max(ret.x, vertices[i].p.x);
        ret.y = std::max(ret.y, vertices[i].p.y);
        ret.z = std::max(ret.z, vertices[i].p.z);
    }
    return ret;
}

int Mesh::findIndexOfVertex(Point3& v)
{
    uint32_t hash = pointHash(v);

    for(unsigned int idx = 0; idx < vertex_hash_map[hash].size(); idx++)
    {
        if ((vertices[vertex_hash_map[hash][idx]].p - v).testLength(vertex_meld_distance))
        {
            return vertex_hash_map[hash][idx];
        }
    }
    vertex_hash_map[hash].push_back(vertices.size());
    vertices.emplace_back(v);
    return vertices.size() - 1;
}

/*!
Returns the index of the 'other' face connected to the edge between vertices with indices idx0 and idx1.
In case more than two faces are connected via the same edge, the next face in a counter-clockwise ordering (looking from idx1 to idx0) is returned.

\cond DOXYGEN_EXCLUDE
    [NON-RENDERED COMENTS]
    For two faces abc and abd with normals n and m, we have that:
    \f{eqnarray*}{
    n &=& \frac{ab \times ac}{\|ab \times ac\|}     \\
    m &=& \frac{ab \times ad}{\|ab \times ad\|}     \\
    n \times m &=& \|n\| \cdot \|m\| \mathbf{p} \sin \alpha  \\
    && (\mathbf{p} \perp n \wedge \mathbf{p} \perp m) \\
    \sin \alpha &=& \|n \times m \|
    &=& \left\| \frac{(ab \times ac) \times (ab \times ad)}{\|ab \times ac\| \cdot \|ab \times ad\|}  \right\|    \\
    &=& \left\| \frac{ (ab \cdot (ac \times ad)) ab  }{\|ab \times ac\| \cdot \|ab \times ad\|}  \right\|    \\
    &=&  \frac{ (ab \cdot (ac \times ad)) \left\| ab   \right\| }{\|ab\| \|ac\| \sin bac \cdot \|ab\| \|ad\| \sin bad}    \\
    &=&  \frac{  ab \cdot (ac \times ad)  }{\|ab\| \|ac\| \|ad\|  \sin bac \sin bad}    \\
    \f}}
\endcond

See <a href="http://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors">Direct way of computing clockwise angle between 2 vectors</a>


*/
int Mesh::getFaceIdxWithPoints(int idx0, int idx1, int notFaceIdx)
{
    std::vector<int> candidateFaces; // in case more than two faces meet at an edge, multiple candidates are generated
    int notFaceVertexIdx = -1; // index of the third vertex of the face corresponding to notFaceIdx
    for(int f : vertices[idx0].connected_faces) // search through all faces connected to the first vertex and find those that are also connected to the second
    {
        if (f == notFaceIdx)
        {
            for (int i = 0; i<3; i++) // find the vertex which is not idx0 or idx1
                if (faces[f].vertex_index[i] != idx0 && faces[f].vertex_index[i] != idx1)
                    notFaceVertexIdx = faces[f].vertex_index[i];
            continue;
        }
        if ( faces[f].vertex_index[0] == idx1 // && faces[f].vertex_index[1] == idx0 // next face should have the right direction!
          || faces[f].vertex_index[1] == idx1 // && faces[f].vertex_index[2] == idx0
          || faces[f].vertex_index[2] == idx1 // && faces[f].vertex_index[0] == idx0
            )  candidateFaces.push_back(f);

    }

    if (candidateFaces.size() == 0) { cura::logError("Couldn't find face connected to face %i.\n", notFaceIdx); return -1; }
    if (candidateFaces.size() == 1) { return candidateFaces[0]; }


    if (notFaceVertexIdx < 0) { cura::logError("Couldn't find third point on face %i.\n", notFaceIdx); return -1; }

    if (candidateFaces.size() % 2 == 0) cura::log("Warning! Edge with uneven number of faces connecting it!(%i)\n", candidateFaces.size()+1);

    FPoint3 vn = vertices[idx1].p - vertices[idx0].p;
    FPoint3 n = vn / vn.vSize(); // the normal of the plane in which all normals of faces connected to the edge lie => the normalized normal
    FPoint3 v0 = vertices[idx1].p - vertices[idx0].p;

// the normals below are abnormally directed! : these normals all point counterclockwise (viewed from idx1 to idx0) from the face, irrespective of the direction of the face.
    FPoint3 n0 = FPoint3(vertices[notFaceVertexIdx].p - vertices[idx0].p).cross(v0);

    double smallestAngle = 1000; // more then 2 PI (impossible angle)
    int bestIdx = -1;

    for (int candidateFace : candidateFaces)
    {
        int candidateVertex;
        {// find third vertex belonging to the face (besides idx0 and idx1)
            for (candidateVertex = 0; candidateVertex<3; candidateVertex++)
                if (faces[candidateFace].vertex_index[candidateVertex] != idx0 && faces[candidateFace].vertex_index[candidateVertex] != idx1)
                    break;
        }

        FPoint3 v1 = vertices[candidateVertex].p -vertices[idx0].p;
        FPoint3 n1 = v1.cross(v0);

        double dot = n0 * n1;
        double det = n * n0.cross(n1);
        double angle = std::atan2(det, dot);
        if (angle < 0) angle += 2*M_PI; // 0 <= angle < 2* M_PI

        if (angle == 0) cura::log("Warning! Overlapping faces: face %i and face %i.\n", notFaceIdx, candidateFace);
        else if (angle < smallestAngle)
        {
            smallestAngle = angle;
            bestIdx = candidateFace;
        }
    }
    if (bestIdx < 0) cura::logError("Couldn't find face connected to face %i.\n", notFaceIdx);
    return bestIdx;
}
