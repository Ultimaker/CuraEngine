#ifndef ADVANCEDSUPPORT_H
#define ADVANCEDSUPPORT_H


#include "modelFile/modelFile.h"

#include "halfEdgeMesh.h"
#include <iostream>

namespace cura {






class SupportChecker
{
    public:

        //! Computes which faces, edges and points need support.
        /*!
        * A face is bad when its angle > maxAngle.
        * An edge is bad when angle is above maxAngle, it is a bottom edge and it is 'below' both connected faces; or when a single connected face is bad; but when both connected faces are bad only if the edge is 'below' both faces.
        * A vertex is bad when all connected vertices have higher z and it is a bottom vertex.
        *
        * Edges within a bad region will only be classified as bad when they need extra support because the edge supports both faces.
        * A bad edge on the boundary of a bad region is classified as bad so that support will be generated up to and including the boundary.
        *
        * A bad vertex in the middle of a bad region needs to be classified as such because we need to support the mid air tip being printed first.
        *
        * \param mesh The mesh from which to verify support requirements.
        * \param maxAngle The max angle (0 is vertical) at which a part can reliably be printed. (0 < maxAngle < .5 pi)
        */
        static SupportChecker getSupportRequireds(Mesh& mesh, double maxAngle);



        double maxAngle;
        double cosMaxAngle;
        double cosMaxAngleNormal; /// == sin maxAngle

        HE_Mesh mesh;

        std::vector<bool> faceIsBad;
        std::vector<bool> edgeIsBad;
        std::vector<bool> vertexIsBad;

        std::vector<Point3> faceNormals;


        SupportChecker(Mesh mmesh, double maxAngleI)
        : maxAngle(maxAngleI)
        , mesh(mmesh)
        , faceIsBad(mesh.faces.size())
        , edgeIsBad(mesh.edges.size())
        , vertexIsBad(mesh.vertices.size())
        , faceNormals(mesh.faces.size())
        {
            cosMaxAngleNormal = cos(maxAngle + .5*M_PI);
            cosMaxAngle = cos(maxAngle);

            faceIsBad.resize(mesh.faces.size());
            edgeIsBad.resize(mesh.edges.size());
            vertexIsBad.resize(mesh.vertices.size());
            faceNormals.resize(mesh.faces.size());
        };

        virtual ~SupportChecker();

        static void testSupportChecker(PrintObject* model);

    protected:
        bool faceNeedsSupport(const HE_Mesh& mesh, int face_idx);
        bool edgeNeedsSupport(const HE_Mesh& mesh, int edge_idx);
        bool vertexNeedsSupport(const HE_Mesh& mesh, int vertex_idx);

    private:
        /*!
        * Check whether edge is 'below' faces; this is similar to convexity, but has more constraints.
        * Edge is below a face if the third point in the face (not in the edge) is above the plane through the edge (AC) and through AD, which is perpendicular to AC and Z (so AD is horizontal).
        * Another way to look at it : an edge is 'below' a face if it is angled down (like the edge to the right in this figure:  L/ ) and the faces make a convex angle.

        B is the third vertex in one adjacent face; B' its vertical projection onto the plane.
        If B' is lower than B, the edge is lower than the face.
        Because AD is perpendicular to AC and Z, we can define it directly in terms of the coordinates of A and AC, and define the plane as:
        \f{eqnarray*}{
        plane &:& (a.x + i * dac.x + j * dac.y, a.y + i * dac.y - j * dac.x, a.z + i * dac.z)   \\
        b'.x &=& b.x    \\
        b'.y &=& b.y    \\
        dab.x &=& i * dac.x + j * dac.y \\
        dab.y &=& i * dac.y - j * dac.x \\
        &=>&    \\
        i &=& \frac{ dac.x * dab.x + dac.y * dab.y }{ dac.x^2 + dac.y^2 } \\
        &=>&    \\
        b'.z &=& a.z + dac.z * i    \\

        && if (dac.x == dac.y) \\
        && then \\
        i &=& \frac{ dab.x + dab.y }{ dac.x + dac.x } \\
        \f}

        \image html edge_overhang.png

        * \param mesh the mesh
        * \param edge the edge to check
        */
        inline bool edgeIsBelowFaces(const HE_Mesh& mesh, const HE_Edge& edge);
        inline bool edgeIsBelowFacesDiagonal(const HE_Mesh& mesh, const HE_Edge& edge, const Point3& a, const Point3& dac); //!< Hlper function for the edge case in which a horizontal line through the plane is perfectly diagonal, in which case the denom parameter of edgeIsBelowFacesNonDiagonal is not finite.
        inline bool edgeIsBelowFacesNonDiagonal(const HE_Mesh& mesh, const HE_Edge& edge, const Point3& a, const Point3& dac, double denom);
};

}//namespace cura


#endif // ADVANCEDSUPPORT_H
