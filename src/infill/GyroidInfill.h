//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "../utils/Coord_t.h"
#include "../settings/EnumSettings.h" //For infill types.
#include "../settings/types/AngleDegrees.h"
#include "../settings/types/Ratio.h"
#include "../utils/IntPoint.h"

namespace cura
{

class Polygons;

class GyroidInfill
{
public:
    GyroidInfill();

    ~GyroidInfill();

    static void generateTotalGyroidInfill(Polygons& result_lines, bool zig_zaggify, coord_t outline_offset, coord_t infill_line_width, coord_t line_distance, const Polygons& in_outline, coord_t z, EFillMethod pattern, const Point& infill_origin, const AngleDegrees fill_angle, const Ratio scaling_z);
    
private:

};

} // namespace cura

