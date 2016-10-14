#ifndef (INFILL_SUBDIVCUBE_H)
#define INFILL_SUBDIVCUBE_H
#include "../sliceDataStorage.h"
namespace cura
{
class Infill;
class SubDivCube
{
public:
    /*!
     * Precompute the octree of subdivided cubes
     * \param mesh contains infill layer data and settings
     */
    static void precomputeOctree(SliceMeshStorage& mesh);
    /*!
     * Generates the lines of subdivision of the specific cube at the specific layer. It recursively calls itself, so it ends up drawing all the subdivision lines of sub-cubes too.
     * \param z the specified layer height
     * \param result (output) The resulting lines
     * \param directionalPots Should be nullptr. Used internally to keep track of line segments that are all pointing the same direction for line segment combining
     */
    void generateSubdivisionLines(int64_t z, Polygons& result, Polygons** directionalPots = nullptr);
private:
    /*!
     * Constructor for SubDivCube. Recursively calls itself eight times to flesh out the octree.
     * \param mesh contains infill layer data and settings
     * \param myCenter the center of the cube
     * \param depth the recursion depth of the cube (0 is most recursed)
     */
    SubDivCube(SliceMeshStorage& mesh, Point3& myCenter, int d);
    /*!
     * Rotates a point 120 degrees about the origin.
     * \param targ the point to rotate.
     */
    static void rot120(Point& targ);
    /*!
     * Rotates a point to align it with the orientation of the infill.
     * \param targ the point to rotate.
     */
    static void initRot(Point& targ);
    /*!
     * Determines if a described theoretical cube should be subdivided based on if a sphere that encloses the cube touches the infill mesh.
     * \param mesh contains infill layer data and settings
     * \param center the center of the described cube
     * \param rad the radius of the enclosing sphere
     */
    static int subDiv(SliceMeshStorage& mesh, Point3& center, int32_t rad);
    /*!
     * Finds the distance to the infill border at the specified layer from the specified point.
     * \param mesh contains infill layer data and settings
     * \param layer_nr the number of the specified layer
     * \param loc the location of the specified point
     * \param distance (output) the distance to the infill border
     * \return Code 0: outside, 1: inside, 2: boundary does not exist at specified layer
     */
    static int distanceFromPointToMesh(SliceMeshStorage& mesh, long int layer_nr, Point& loc, int32_t* distance);
    int d;
    Point3 center;
    SubDivCube *(children[8]) = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    static std::vector<int64_t> sideLen;
    static std::vector<int64_t> height;
    static std::vector<int64_t> squareCutAcross;
    static std::vector<int64_t> maxDrawDiff;//maximum difference from the center level of the cube in the x axis when lines still need to be drawn.
    static std::vector<int64_t> maxLineOffset;//maximum distance from the 2d center that subdividing lines should be drawn at
    static double radMult;
    static int32_t radAdd;
    static double rotCoefX;
    static double rotCoefY;
    static constexpr double sqrt_three_fourths = 0.8660254037844386467637231707529361834714026269051903;
};
extern SubDivCube *baseSubDivCube;
}
#endif //INFILL_SUBDIVCUBE_H
