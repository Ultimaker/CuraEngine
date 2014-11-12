#include "advancedSupport.h"
#include "utils/intpoint.h"
#include <iostream> /* cerr */
#include <math.h> /* isfinite */
#include <cassert>

/// enable/disable debug output
#define ADV_SUP_DEBUG 0

#define ADV_SUP_DEBUG_SHOW(x) do { std::cerr << #x << ": " << x << std::endl; } while (0)
#define ADV_SUP_DEBUG_PRINTLN(x) do { std::cerr <<  x << std::endl; } while (0)
#if ADV_SUP_DEBUG == 1
#  define ADV_SUP_DEBUG_DO(x) do { x } while (0);
#else
#  define ADV_SUP_DEBUG_DO(x)
#endif

namespace cura {

SupportChecker SupportChecker::getSupportRequireds(Mesh& mesh, double maxAngle)
{

    SupportChecker supporter(mesh, maxAngle);
    ADV_SUP_DEBUG_DO(
        std::cerr << "-----------------------\n--getSupportRequireds--\n-----------------------" << std::endl;
        std::cerr << "cosMaxAngle = " << supporter.cosMaxAngle << std::endl;
        std::cerr << "cosMaxAngleNormal = " << supporter.cosMaxAngleNormal << std::endl;
    )

    for (int f = 0 ; f < supporter.mesh.faces.size() ; f++)
    {
        supporter.faceIsBad[f] = supporter.faceNeedsSupport(supporter.mesh, f);
    }

    for (int e = 0 ; e < supporter.mesh.edges.size() ; e++)
    {
        bool bad = supporter.edgeNeedsSupport(supporter.mesh, e);
        supporter.edgeIsBad[e] = bad;
        if (bad)
        {
            supporter.edgeIsBad[supporter.mesh.edges[e].converse_edge_idx] = bad;
        }
    }

    for (int v = 0 ; v < supporter.mesh.vertices.size() ; v++)
    {
        supporter.vertexIsBad[v] = supporter.vertexNeedsSupport(supporter.mesh, v);
    }

    ADV_SUP_DEBUG_DO( std::cerr << "------------------------\n--END SupportRequireds--\n------------------------" << std::endl; )
    return supporter;
}

bool SupportChecker::faceNeedsSupport(const HE_Mesh& mesh, int face_idx)
{
    const HE_Face& face = mesh.faces[face_idx];
    const Point3 normal = mesh.getNormal(face);

    faceNormals[face_idx] = normal;

    double cosAngle = double(normal.z) / double(normal.vSize()); // fabs


    if (cosAngle < cosMaxAngleNormal) return true;
    else return false;
}

/// supposes all faces have already been checked
bool SupportChecker::edgeNeedsSupport(const HE_Mesh& mesh, int edge_idx)
{
    ADV_SUP_DEBUG_DO( std::cerr << "edge " << edge_idx << " : "; )
    const HE_Edge& edge = mesh.edges[edge_idx];
    const HE_Edge& backEdge = mesh.edges[edge.converse_edge_idx];

    if (edgeIsBad[edge.converse_edge_idx])
    {
        ADV_SUP_DEBUG_DO( std::cerr << " converse is bad" << std::endl; )
        return true;
    }

    bool bad1 = faceIsBad[edge.face_idx];
    bool bad2 = faceIsBad[backEdge.face_idx];


    if (bad1 != bad2) /// xor : one bad one not... the edge of a bad area is always bad
    {
        ADV_SUP_DEBUG_DO( std::cerr << " on one bad face" << std::endl;)
        return true;
    }



    if (!bad1 && !bad2) /// == !(bad1 && bad2) , since (bad1 != bad2) is already checked above
    { /// check if angle with Z-axis is great enough to require support
        Point3 vect = mesh.getTo(edge).p - mesh.getFrom(edge).p;
        double absCosAngle = fabs( (double)vect.z / (double)vect.vSize() );


        ADV_SUP_DEBUG_DO( std::cerr << " (absCosAngle = " << absCosAngle << ")" << std::endl; )

        if (absCosAngle > cosMaxAngle)
        {
            ADV_SUP_DEBUG_DO( std::cerr << " absCosAngle > cosMaxAngle ; vect.z = " <<vect.z << " ; vect.vSize() = " << vect.vSize() << std::endl;)
            return false;
        }
    }


    { /// check if edge is top edge (normal pointing up)
        const Point3 face_1_normal = faceNormals[edge.face_idx];
        const Point3 face_2_normal = faceNormals[mesh.getConverse(edge).face_idx];

        int32_t zNormal = face_1_normal.z / face_1_normal.vSize() + face_2_normal.z / face_2_normal.vSize(); /// (unnormalized)

        if (zNormal > 0)
        {
            ADV_SUP_DEBUG_DO( std::cerr << "edge is top edge" << std::endl; )
            return false; /// edge is a top edge
        }
    }

    return edgeIsBelowFaces(mesh, edge);


}

bool SupportChecker::edgeIsBelowFaces(const HE_Mesh& mesh, const HE_Edge& edge)
{
    Point3 a = mesh.getFrom(edge).p;
    Point3 dac =  mesh.getTo(edge).p - a;
    if (dac.x ==0 && dac.y ==0)
    {
        ADV_SUP_DEBUG_DO( std::cerr << "edge is vertical" << std::endl; )
        return false;
    }
    double denom = 1. / (dac.x*dac.x - dac.y*dac.y);


    if (!std::isfinite(denom)) /// then |dac.x| == |dac.y|
    {
        return edgeIsBelowFacesDiagonal(mesh, edge, a, dac);
    } else
    {
        return edgeIsBelowFacesNonDiagonal(mesh, edge, a, dac, denom);
    }
}

bool SupportChecker::edgeIsBelowFacesDiagonal(const HE_Mesh& mesh, const HE_Edge& edge, const Point3& a, const Point3& dac)
{
    assert(dac.x == dac.y || dac.x == -dac.y);

    ADV_SUP_DEBUG_DO( std::cerr << "edge is diagonal" << std::endl; )

    short sign = (dac.x == -dac.y)? -1 : 1;
    double denom = 1. / (2 * dac.x);

    { /// first face
        Point3 b = mesh.getTo(mesh.getNext(edge)).p;
        Point3 dab =  b - a;

        double projectedZb = a.z + dac.z * (dab.x + sign * dab.y) * denom;

        if (b.z < projectedZb)
        {
            ADV_SUP_DEBUG_DO(
                std::cerr << "edge is above first face" << std::endl;
                std::cerr << "denom = " << denom << std::endl;
                std::cerr << "projectedZb = " << projectedZb << std::endl;
                std::cerr << "b.z  = " << b.z  << std::endl;
            )
            return false;
        }
    }
    { /// second face
        Point3 b2 = mesh.getTo(mesh.getNext(mesh.getConverse(edge))).p;
        Point3 dab2 = b2 - a;

        double projectedZb2 = a.z + dac.z * (dab2.x + sign * dab2.y) * denom;


        if (b2.z < projectedZb2)
        {
            ADV_SUP_DEBUG_DO( std::cerr << "edge is above second face" << std::endl; )
            return false;
        } else {
            ADV_SUP_DEBUG_DO( std::cerr << "edge lower than both faces" << std::endl; )
            return true; /// edge is 'lower' than both faces
        }
    }
}



bool SupportChecker::edgeIsBelowFacesNonDiagonal(const HE_Mesh& mesh, const HE_Edge& edge, const Point3& a, const Point3& dac, double denom)
{
    { /// first face
        Point3 b = mesh.getTo(mesh.getNext(edge)).p;
        Point3 dab =  b - a;

        double projectedZb = a.z + dac.z * (dac.x*dab.x - dac.y*dab.y) * denom;

        if (b.z < projectedZb)
        {
            ADV_SUP_DEBUG_DO(
                std::cerr << "edge is above first face" << std::endl;
                std::cerr << "denom = " << denom << std::endl;
                std::cerr << "projectedZb = " << projectedZb << std::endl;
                std::cerr << "b.z  = " << b.z  << std::endl;
            )
            return false;
        }
    }

    { /// second face
        Point3 b2 = mesh.getTo(mesh.getNext(mesh.getConverse(edge))).p;
        Point3 dab2 = b2 - a;

        double projectedZb2 = a.z + dac.z * (dac.x*dab2.x - dac.y*dab2.y) * denom;


        if (b2.z < projectedZb2)
        {
            ADV_SUP_DEBUG_DO(
                std::cerr << "edge is above second face" << std::endl;
                std::cerr << "denom = " << denom << std::endl;
                std::cerr << "projectedZb = " << projectedZb << std::endl;
                std::cerr << "b.z  = " << b.z  << std::endl;
            )
            return false;
        } else {
            ADV_SUP_DEBUG_DO( std::cerr << "edge lower than both faces" << std::endl; )
            return true; /// edge is 'lower' than both faces
        }
    }
}

bool SupportChecker::vertexNeedsSupport(const HE_Mesh& mesh, int vertex_idx)
{
    ADV_SUP_DEBUG_DO( std::cerr << "vertex " << vertex_idx << " : "; )
    const HE_Vertex& vertex = mesh.vertices[vertex_idx];
    const HE_Edge& someEdge = mesh.getSomeEdge(vertex);

    if (faceNormals[someEdge.face_idx].z > 0)
    {
        return false; /// vertex can at most be the bottom of a concave dimple
    }

    bool first = true;
    for (const HE_Edge* departing_edge = &someEdge ; first || departing_edge != &someEdge ; departing_edge = &mesh.getNext(mesh.getConverse(*departing_edge)) )
    {
        ADV_SUP_DEBUG_DO( std::cerr << " v" << departing_edge->to_vert_idx; )
        if (mesh.getTo(*departing_edge).p.z < vertex.p.z) return false;
        first = false;
    }
    ADV_SUP_DEBUG_DO( std::cerr << std::endl << " vertex is bad" << std::endl; )

    return true;
}

void SupportChecker::testSupportChecker(PrintObject* model)
{
    std::cerr << "=============================================\n" << std::endl;

    for (int mi = 0 ; mi < model->meshes.size() ; mi++)
    {
        //HE_Mesh mesh(model->meshes[mi]);
        SupportChecker supporter = SupportChecker::getSupportRequireds(model->meshes[mi], .785); // 45/180*M_PI


        std::cerr << "faces badness:" << std::endl;
        for (int f = 0 ; f < supporter.mesh.faces.size() ; f++)
        {
            std::cerr << f << " " << (supporter.faceIsBad[f]? "TRUE" : "f") << std::endl;
        }

        std::cerr << "edges badness:" << std::endl;
        for (int e = 0 ; e < supporter.mesh.edges.size() ; e++)
        {
            std::cerr << e << " " << (supporter.edgeIsBad[e]? "TRUE" : "f") << std::endl;
        }

        std::cerr << "vertices badness:" << std::endl;
        for (int v = 0 ; v < supporter.mesh.vertices.size() ; v++)
        {
            std::cerr << v << " " << (supporter.vertexIsBad[v]? "TRUE" : "f") << std::endl;
        }
        //for (int f = 0; f < mesh.faces.size(); f++)
        //    std::cerr << mesh.faces[f].cosAngle() << std::endl;
    }
    std::cerr << "=============================================\n" << std::endl;

}

SupportChecker::~SupportChecker()
{
    //dtor
}




} // namespace cura
