#ifndef HALFEDGEMESH_H
#define HALFEDGEMESH_H

#include "mesh.h"


/**
 "f" is short for face
 "e" is short for edge
 "v" is short for vertex
*/


//     /|\ |   _
//      |  |   [ front of face ]
//      | \|/  "
/// the face is always on the left hand side of the half-edge

class HE_Vertex;
class HE_Face;
class HE_Edge;

class HE_Mesh
{
    public:
        std::vector<HE_Vertex> vertices;
        std::vector<HE_Face> faces;
        std::vector<HE_Edge> edges;

        HE_Mesh(Mesh& mesh);
        virtual ~HE_Mesh();


        const HE_Edge& getNext(const HE_Edge& edge)  const ;
        const HE_Edge& getPrev(const HE_Edge& edge)  const ;
        const HE_Edge& getConverse(const HE_Edge& edge)  const ;
        const HE_Vertex& getTo(const HE_Edge& edge)  const ;
        const HE_Vertex& getFrom(const HE_Edge& edge)  const ;

        const HE_Edge& getSomeEdge(const HE_Vertex& vertex) const  ;


        Point3 getNormal(const HE_Face& face) const;

    protected:
    private:
};

class HE_Edge {
    public:
        int from_vert_idx;
        int to_vert_idx;

        int next_edge_idx;
        int prev_edge_idx;

        int converse_edge_idx;

        int face_idx;

        HE_Edge(int from_vert_idx, int to_vert_idx, int face_idx);

        std::string toString() {
            return "from vertex "+ std::to_string(from_vert_idx) + " to "+ std::to_string(to_vert_idx)+" from edge " + std::to_string(prev_edge_idx)+" to "+std::to_string(next_edge_idx)+" of face"+std::to_string(face_idx)+"; converse of "+std::to_string(converse_edge_idx) ;
            };

};
class HE_Vertex {
    public:
        Point3 p;
        //std::vector<HE_Edge> edges;
        int someEdge_idx;
        HE_Vertex(Point3 p, int someEdge_idx);

        std::string toString() { return "p"+std::to_string(p.x)+","+std::to_string(p.y)+","+std::to_string(p.z) + " - someEdge: "+std::to_string(someEdge_idx); };
};
class HE_Face {
    public:
        int edge_index[3];

        std::string toString() {return "edges: "+std::to_string(edge_index[0])+", "+std::to_string(edge_index[1])+", "+std::to_string(edge_index[2]); };
        HE_Face();

//        inline double cosAngle()
//        {
//            Point3 normal = getNormal();
//            return double(normal.z) / double(normal.vSize()); // fabs
//        };
};


#endif // HALFEDGEMESH_H
