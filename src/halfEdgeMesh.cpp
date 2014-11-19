#include "halfEdgeMesh.h"
#include "utils/logoutput.h"


/// enable/disable debug output
#define HE_MESH_DEBUG 1

#define HE_MESH_DEBUG_SHOW(x) do { std::cerr << #x << ": " << x << std::endl; } while (0)
#define HE_MESH_DEBUG_PRINTLN(x) do { std::cerr <<  x << std::endl; } while (0)
#if HE_MESH_DEBUG == 1
#  define HE_MESH_DEBUG_DO(x) do { x } while (0);
#else
#  define HE_MESH_DEBUG_DO(x)
#endif



#include <iostream>

//#include "easylogging++.h"



const HE_Edge& HE_Mesh::getNext(const HE_Edge& edge) const { return edges[edge.next_edge_idx]; }
const HE_Edge& HE_Mesh::getPrev(const HE_Edge& edge) const { return edges[edge.prev_edge_idx]; }
const HE_Edge& HE_Mesh::getConverse(const HE_Edge& edge) const { return edges[edge.converse_edge_idx]; }
const HE_Vertex& HE_Mesh::getTo(const HE_Edge& edge) const { return vertices[edge.to_vert_idx]; }
const HE_Vertex& HE_Mesh::getFrom(const HE_Edge& edge) const { return vertices[edge.from_vert_idx]; }

const HE_Edge& HE_Mesh::getSomeEdge(const HE_Vertex& vertex) const { return edges[vertex.someEdge_idx]; }

Point3 HE_Mesh::getNormal(const HE_Face& face) const
{
    Point3 p0 = vertices[edges[face.edge_index[0]].from_vert_idx].p;
    Point3 p1 = vertices[edges[face.edge_index[0]].to_vert_idx].p;
    Point3 p2 = vertices[edges[face.edge_index[2]].from_vert_idx].p;
    return FPoint3::cross(p1-p0, p2-p0).normalized().toPoint3();
};

HE_Vertex::HE_Vertex(Point3 p, int someEdge_idx)
    : p(p)
    , someEdge_idx(someEdge_idx)
    {};

HE_Edge::HE_Edge(int from_vert_idx, int to_vert_idx, int face_idx)
    : from_vert_idx(from_vert_idx)
    , to_vert_idx(to_vert_idx)
    , face_idx(face_idx)
    {};

HE_Face::HE_Face()
    {};

HE_Mesh::HE_Mesh(Mesh& mesh)
{

    for (int vIdx = 0 ; vIdx < mesh.vertices.size() ; vIdx++)
    {
        vertices.push_back(HE_Vertex(mesh.vertices[vIdx].p, -1));
    }

    for (int fIdx = 0 ; fIdx < mesh.faces.size() ; fIdx++)
    {
        MeshFace& face = mesh.faces[fIdx];
        HE_Face heFace;
        HE_Edge edge0(face.vertex_index[0], face.vertex_index[1], fIdx); /// vertices in face are ordered counter-clockwise
        HE_Edge edge1(face.vertex_index[1], face.vertex_index[2], fIdx);
        HE_Edge edge2(face.vertex_index[2], face.vertex_index[0], fIdx);

        int newEdgeIndex = edges.size();

        vertices[ face.vertex_index[0] ].someEdge_idx = newEdgeIndex+0; /// overwrites existing data, if present
        vertices[ face.vertex_index[1] ].someEdge_idx = newEdgeIndex+1;
        vertices[ face.vertex_index[2] ].someEdge_idx = newEdgeIndex+2;

        edge0.next_edge_idx = newEdgeIndex+1;
        edge1.next_edge_idx = newEdgeIndex+2;
        edge2.next_edge_idx = newEdgeIndex+0;

        edge0.prev_edge_idx = newEdgeIndex+2;
        edge1.prev_edge_idx = newEdgeIndex+0;
        edge2.prev_edge_idx = newEdgeIndex+1;

        edges.push_back(edge0);
        edges.push_back(edge1);
        edges.push_back(edge2);

        heFace.edge_index[0] = newEdgeIndex+0;
        heFace.edge_index[1] = newEdgeIndex+1;
        heFace.edge_index[2] = newEdgeIndex+2;
        faces.push_back(heFace);

    }


    /// connect half-edges:

    bool faceEdgeIsConnected[mesh.faces.size()][3] = {}; /// initialize all as false


    /// for each edge of each face : if it doesn't have a converse then find the converse in the edges of the opposite face
    for (int fIdx = 0 ; fIdx < mesh.faces.size() ; fIdx++)
    {
        MeshFace& face = mesh.faces[fIdx];

        for (int eIdx = 0; eIdx < 3; eIdx++)
        {
            if (!faceEdgeIsConnected[fIdx][eIdx])
            {
                int edge_index = faces[fIdx].edge_index[eIdx];
                HE_Edge& edge = edges[ edge_index ];
                int face2 = face.connected_face_index[eIdx]; /// connected_face X is connected via vertex X and vertex X+1

                if (face2 < 0)
                {
                    cura::logError("Incorrect model: disconnected faces. Support generation aborted.\n");
                    exit(1); /// TODO: not exit, but continue without support!
                }

                for (int e2 = 0; e2 < 3; e2++)
                {
                    if (mesh.faces[face2].vertex_index[e2] == edge.to_vert_idx)
                    {
                        edges[ faces[face2].edge_index[e2] ].converse_edge_idx = edge_index;
                        edge.converse_edge_idx = faces[face2].edge_index[e2];
                        faceEdgeIsConnected[face2][e2] = true; /// the other way around doesn't have to be set; we will not pass the same edge twice
                        break;
                    }
                    if (e2 == 2) std::cerr << "Couldn't find converse of edge " << std::to_string(edge_index) <<"!!!!!" << std::endl;
                }

            }
        }

    }

    HE_MESH_DEBUG_DO(
        std::cerr <<  "============================" << std::endl;
        std::cerr <<  "mesh: " << std::endl;
        std::cerr <<  "faces: "+ std::to_string(mesh.faces.size()) << std::endl;
        std::cerr << "vertices: "+std::to_string(mesh.vertices.size()) << std::endl;


        std::cerr <<  "============================" << std::endl;
        std::cerr <<  ("half-edge mesh: ") << std::endl;
        std::cerr <<  ("faces: ") << std::endl;
        for (int f = 0; f < faces.size(); f++)
            std::cerr << f << " " <<(faces[f].toString()) << std::endl;
        std::cerr << ("edges: ") << std::endl;
        for (int f = 0; f < edges.size(); f++)
            std::cerr << f << " " <<(edges[f].toString()) << std::endl;
        std::cerr << ("vertices: ") << std::endl;
        for (int f = 0; f < vertices.size(); f++)
            std::cerr << f << " " <<(vertices[f].toString()) << std::endl;
        std::cerr <<  "============================" << std::endl;
     )

}

HE_Mesh::~HE_Mesh()
{
    //dtor
}


