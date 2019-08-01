//Copyright (c) 2019 Ultimaker B.V.


#ifndef GCODER_H
#define GCODER_H

#include <fstream>

#include "utils/polygon.h"
#include "utils/ExtrusionSegment.h"
#include "utils/ExtrusionJunction.h"

namespace arachne
{

/*!
 * Write gcode for a single layer variable line width print
 */
class GcodeWriter
{
public:
    GcodeWriter(std::string filename, int type, coord_t layer_thickness = MM2INT(0.3), float print_speed = 300, float travel_speed = 3500, float extrusion_multiplier = 0.5);
    ~GcodeWriter();
    static constexpr int type_P3 = 0;
    static constexpr int type_UM3 = 1;
    void print(std::vector<std::vector<std::vector<ExtrusionJunction>>>& polygons_per_index, std::vector<std::vector<std::vector<ExtrusionJunction>>>& polylines_per_index, AABB aabb);
    void move(Point p);
    void print(ExtrusionJunction from, ExtrusionJunction to);
private:
    void printSingleExtrusionMove(ExtrusionJunction& from, ExtrusionJunction& to);
    std::ofstream file;
    int type;
    Point build_plate_middle = Point(MM2INT(100), MM2INT(150));
    float filament_diameter = 2.85;
    coord_t discretization_size = MM2INT(0.4);
    coord_t nozzle_size = MM2INT(0.4);

    coord_t layer_thickness;
    float print_speed;
    float travel_speed;
    float extrusion_multiplier;

    Point reduction;

    Point cur_pos;
    float last_E = 0;

    float getExtrusionFilamentMmPerMmMove(coord_t width);
};




} // namespace arachne
#endif // GCODER_H
