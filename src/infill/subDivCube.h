#ifndef SUBDIVCUBE_H
#define SUBDIVCUBE_H
#include "../sliceDataStorage.h"
namespace cura
{
class Infill;
class SubDivCube
{
public:
    static void init(SliceMeshStorage& gMesh);
    void draw(int64_t z, Polygons& result, Polygons** dir);
private:
    static constexpr double sqrt_three_fourths = 0.8660254037844386467637231707529361834714026269051903;//this is used for the 120 degree rotation...
    SubDivCube(SliceMeshStorage& mesh, Point3& myCenter, int d);
    static std::vector<int64_t> sideLen;//length of cube side
    static std::vector<int64_t> height;//height of cube
    static std::vector<int64_t> squareCutAcross;//length across face of cube
    static std::vector<int64_t> maxDrawDiff;//maximum difference from the center level of the cube in the x axis when lines still need to be drawn.
    static std::vector<int64_t> maxLineOffset;//maximum distance from the 2d center that subdividing lines should be drawn at
    static double radMult;
    static int32_t radAdd;
    int d;
    Point3 center;//location of cube center
    SubDivCube *(children[8]) = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
    static int dist(SliceMeshStorage& mesh, long int layer_nr, Point& loc, int32_t* dist);//returns 1 if inside, distance from polygon put into dist
    static int subDiv(SliceMeshStorage& mesh, Point3& center, int32_t sideLen, int32_t rad);
    static void rot120(Point& targ);
    static void initRot(Point& targ);
    static double rotCoefX;
    static double rotCoefY;
};
extern SubDivCube *baseSubDivCube;//the big cube that encompasses everything
}
#endif
