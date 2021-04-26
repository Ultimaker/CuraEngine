//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "../utils/Coord_t.h"
#include "../utils/AABB.h"
#include "../utils/polygon.h"

namespace cura
{

class Polygons;

class TroctInfill
{
public:
  TroctInfill();

  ~TroctInfill();
  
  static void generateTotalTroctInfill(Polygons& result_lines,
				       bool zig_zaggify, coord_t outline_offset,
				       coord_t infill_line_width,
				       coord_t line_distance,
				       const Polygons& in_outline, coord_t z);
    
private:

  static float triWave(float pos, float gridSize);
  static float troctWave(float pos, float gridSize, float Zpos);
  static std::vector<float> getCriticalPoints(float Zpos, float gridSize);
  static std::vector<float> colinearPoints(const float Zpos, float gridSize,
					   std::vector<float> critPoints,
					   const size_t boundsMin,
					   const size_t boundsMax);
  static std::vector<float> perpendPoints(const float Zpos, float gridSize,
					  std::vector<float> critPoints,
					  const size_t baseLocation,
					  const size_t boundsMin,
					  const size_t boundsMax, float perpDir);
  static void zip(PolygonRef result,
		  const std::vector<float> &x,
		  const std::vector<float> &y);
  static Polygons makeGrid(float Zpos, float gridSize, AABB bounds);

};

} // namespace cura

