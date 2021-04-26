//Copyright (c) 2021 David Eccles (gringer)
//CuraEngine is released under the terms of the AGPLv3 or higher.

/*
Creates a contiguous sequence of points at a specified height that make
up a horizontal slice of the edges of a space filling truncated
octahedron tesselation. The octahedrons are oriented so that the
square faces are in the horizontal plane with edges parallel to the X
and Y axes.

Note: this code is derived from David Eccles' truncated octahedron
infill (3D honeycomb) from Slic3r.
*/

#include "TroctInfill.h"
#include "../utils/AABB.h"
#include "../utils/linearAlg2D.h"
#include "../utils/polygon.h"

namespace cura {

TroctInfill::TroctInfill() {
}

TroctInfill::~TroctInfill() {
}

// sign function
template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}
  
// triangular wave function
// this has period (gridSize * 2), and amplitude (gridSize / 2),
// with triWave(pos = 0) = 0
float TroctInfill::triWave(float pos, float gridSize)
{
  float t = (pos / (gridSize * 2.)) + 0.25; // convert relative to grid size
  t = t - (int)t; // extract fractional part
  return((1. - abs(t * 8. - 4.)) * (gridSize / 4.) + (gridSize / 4.));
}

// truncated octagonal waveform, with period and offset
// as per the triangular wave function. The Z position adjusts
// the maximum offset [between -(gridSize / 4) and (gridSize / 4)], with a
// period of (gridSize * 2) and troctWave(Zpos = 0) = 0
float TroctInfill::troctWave(float pos, float gridSize, float Zpos)
{
  float Zcycle = triWave(Zpos, gridSize);
  float perpOffset = Zcycle / 2;
  float y = triWave(pos, gridSize);
  return((abs(y) > abs(perpOffset)) ?
	 (sgn(y) * perpOffset) :
	 (y * sgn(perpOffset)));
}

// Identify the important points of curve change within a truncated
// octahedron wave (as waveform fraction t):
// 1. Start of wave (always 0.0)
// 2. Transition to upper "horizontal" part
// 3. Transition from upper "horizontal" part
// 4. Transition to lower "horizontal" part
// 5. Transition from lower "horizontal" part
/*    o---o
 *   /     \
 * o/       \
 *           \       /
 *            \     /
 *             o---o
 */
std::vector<float> TroctInfill::getCriticalPoints(float Zpos, float gridSize)
{
  std::vector<float> res = {0.};
  float perpOffset = abs(triWave(Zpos, gridSize) / 2.);

  float normalisedOffset = perpOffset / gridSize;
  // note: 0 == straight line
  if(normalisedOffset > 0){
    res.push_back(gridSize * (0. + normalisedOffset));
    res.push_back(gridSize * (1. - normalisedOffset));
    res.push_back(gridSize * (1. + normalisedOffset));
    res.push_back(gridSize * (2. - normalisedOffset));
  }
  return(res);
}

// Generate an array of points that are in the same direction as the
// basic printing line (i.e. Y points for columns, X points for rows)
// Note: a negative offset only causes a change in the perpendicular
// direction
std::vector<float> TroctInfill::colinearPoints(const float Zpos, float gridSize,
					       std::vector<float> critPoints,
					       const size_t boundsMin,
					       const size_t boundsMax)
{
  std::vector<float> points;
  points.push_back(boundsMin);
  for (float cLoc = boundsMin; cLoc < boundsMax; cLoc+= (gridSize * 2)) {
    for(size_t pi = 0; pi < critPoints.size(); pi++){
      points.push_back(boundsMin + cLoc + critPoints[pi]);
    }
  }
  points.push_back(boundsMax);
  return points;
}

// Generate an array of points for the dimension that is perpendicular to
// the basic printing line (i.e. X points for columns, Y points for rows)
std::vector<float> TroctInfill::perpendPoints(const float Zpos, float gridSize,
					      std::vector<float> critPoints,
					      const size_t baseLocation,
					      const size_t boundsMin,
					      const size_t boundsMax, float perpDir)
{
  std::vector<float> points;
  points.push_back(baseLocation);
  for (float cLoc = boundsMin; cLoc < boundsMax; cLoc+= (gridSize * 2)) {
    for(size_t pi = 0; pi < critPoints.size(); pi++){
      float offset = troctWave(critPoints[pi], gridSize, Zpos);
      points.push_back(baseLocation + (offset * perpDir));
    }
  }
  points.push_back(baseLocation);
  return points;
}

void TroctInfill::zip(PolygonRef result,
		      const std::vector<float> &x,
		      const std::vector<float> &y)
{
    assert(x.size() == y.size());
    for (size_t i = 0; i < x.size(); ++ i){
      Point current = Point(x[i], y[i]);
      result.add(current);
    }
}

// Generate a set of curves (array of array of 2d points) that
// describe a horizontal slice of a truncated regular octahedron with
// a specified grid square size. gridWidth and gridHeight define the
// width and height of the bounding box respectively
Polygons TroctInfill::makeGrid(float Zpos, float gridSize,
			       const AABB bounds)
{
  Polygons result;
  std::vector<float> critPoints = getCriticalPoints(Zpos, gridSize);
  float zCycle = fmod(Zpos + gridSize/2, gridSize * 2.) / (gridSize * 2.);
  bool printVert = zCycle < 0.5;
  if (printVert) {
    int perpDir = -1;
    for (float x = bounds.min.X; x <= (bounds.max.X); x+= gridSize, perpDir *= -1) {
      PolygonRef newPoints = result.newPoly();
      zip(newPoints,
	  perpendPoints(Zpos, gridSize, critPoints, x,
			bounds.min.Y, bounds.max.Y, perpDir), 
	  colinearPoints(Zpos, gridSize, critPoints, bounds.min.Y, bounds.max.Y));
      if (perpDir == 1)
	newPoints.reverse();
    }
  } else {
    int perpDir = 1;
    for (float y = bounds.min.Y + gridSize;
	 y <= (bounds.max.Y); y+= gridSize, perpDir *= -1) {
      PolygonRef newPoints = result.newPoly();
      zip(newPoints,
	  colinearPoints(Zpos, gridSize, critPoints, bounds.min.X, bounds.max.X),
	  perpendPoints(Zpos, gridSize, critPoints, y,
			bounds.min.X, bounds.max.X, perpDir));
      if (perpDir == -1)
	newPoints.reverse();
    }
  }
  return result;
}

void TroctInfill::
generateTotalTroctInfill(
			 Polygons& result_lines, bool zig_zaggify,
			 coord_t outline_offset, coord_t infill_line_width,
			 coord_t line_distance, const Polygons& in_outline,
			 coord_t z)
{

    const Polygons outline = in_outline.offset(outline_offset);
    const AABB aabb(outline);

    // Note: with equally-scaled X/Y/Z, the pattern will create a vertically-stretched
    // truncated octahedron; so Z is pre-adjusted first by scaling by sqrt(2)
    float zScale = sqrt(2);
    float estDensity = infill_line_width / line_distance; // assumed

    // adjustment to account for the additional distance of octagram curves
    // note: this only strictly applies for a rectangular area where the total
    //       Z travel distance is a multiple of the spacing
    // = 4 * integrate(func=4*x(sqrt(2) - 1) + 1, from=0, to=0.25)
    // = (sqrt(2) + 1) / 2 [... I think]
    // make a first guess at the preferred grid Size
    float gridSize = (line_distance * ((zScale + 1.) / 2.));

    // This density calculation is incorrect for many values > 25%, possibly
    // due to quantisation error, so this value is used as a first guess, then the
    // Z scale is adjusted to make the layer patterns consistent / symmetric
    // This means that the resultant infill won't be an ideal truncated octahedron,
    // but it should look better than the equivalent quantised version
    
    float layerHeight = infill_line_width; // assume width == height
    // ceiling to an integer value of layers per Z
    // (with a little nudge in case it's close to perfect)
    float layersPerModule = floor((gridSize * 2) / (zScale * layerHeight) + 0.05);
    if(estDensity > 0.42){ // exact layer pattern for >42% density
      layersPerModule = 2;
      // re-adjust the grid size for a partial octahedral path
      // (scale of 1.1 guessed based on modeling)
      gridSize = (line_distance * 1.1);
      // re-adjust zScale to make layering consistent
      zScale = (gridSize * 2) / (layersPerModule * layerHeight);
    } else {
      if(layersPerModule < 2){
	layersPerModule = 2;
      }
      // re-adjust zScale to make layering consistent
      zScale = (gridSize * 2) / (layersPerModule * layerHeight);
      // re-adjust the grid size to account for the new zScale
      gridSize = (line_distance * ((zScale + 1.) / 2.));
      // re-calculate layersPerModule and zScale
      layersPerModule = floor((gridSize * 2) / (zScale * layerHeight) + 0.05);
      if(layersPerModule < 2){
	layersPerModule = 2;
      }
      zScale = (gridSize * 2) / (layersPerModule * layerHeight);
    }

    Polygons result = makeGrid(z * zScale, gridSize, aabb);
    result_lines = result;
}

} // namespace cura
