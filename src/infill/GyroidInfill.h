//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "../utils/Coord_t.h"

namespace cura
{

class Polygons;

class GyroidInfill
{
public:
    GyroidInfill();

    ~GyroidInfill();

    static void generateTotalGyroidInfill(Polygons& result_lines, bool zig_zaggify, coord_t outline_offset, coord_t infill_line_width, coord_t line_distance, const Polygons& in_outline, coord_t z);
    
private:

};

} // namespace cura

