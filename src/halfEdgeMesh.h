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
// the face is always on the left hand side of the half-edge

class HE_Vertex;
class HE_Face;
class HE_Edge;

/*!
Half-edge mesh - a more elaborate data structure to store a mesh.

Each edge is represented by (in general) two directed half-edges.
The next and prev edge are the other edges within the face.
The converse is the half-edge directed in the opposite direction, belonging to the face on the other side of the edge.

Note that a correct model may have more than 2 faces connected to a single edge!
The converse of a half-edge of such an edge belonging to face F is given by the half-edge belonging to face F2, connected to F via the outside; see ASCII art below:

: horizontal slice through vertical edge connected to four faces :

\verbatim
[inside] x|
         x| <--+--- faces with half-edges being each others converse
   xxxxxxx|   \|/
   -------+-------
      ^   |xxxxxxx
      +-->|x
      |   |x [inside]
      |
    faces with half-edges being each others converse
\endverbatim

As such we should keep in mind that when starting from some half-edge connected to a vertex, we cannot be guaranteed to be able to traverse all connected edges just by using the operations getNext() and getConverse()!
Walking along the surface of a model means walking along the outside of the model (as opposed to the inside).

*/
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
