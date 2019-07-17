//Copyright (c) 2019 Ultimaker B.V.


#include "GcodeWriter.h"


namespace arachne
{
GcodeWriter::GcodeWriter(std::string filename, int type, coord_t layer_thickness, float print_speed, float travel_speed)
: file(filename.c_str())
, type(type)
, layer_thickness(layer_thickness)
, print_speed(print_speed)
, travel_speed(travel_speed)
{
    assert(file.good());

    file << ";START_OF_HEADER\n";
    file << ";HEADER_VERSION:0.1\n";
    file << ";FLAVOR:Griffin\n";
    file << ";GENERATOR.NAME:libArachne\n";
    file << ";GENERATOR.VERSION:3.1.0\n";
    file << ";GENERATOR.BUILD_DATE:2017-12-05\n";
    file << ";TARGET_MACHINE.NAME:Ultimaker 3\n";
    file << ";EXTRUDER_TRAIN.0.INITIAL_TEMPERATURE:215\n";
    file << ";EXTRUDER_TRAIN.0.MATERIAL.VOLUME_USED:9172\n";
    file << ";EXTRUDER_TRAIN.0.MATERIAL.GUID:506c9f0d-e3aa-4bd4-b2d2-23e2425b1aa9\n";
    file << ";EXTRUDER_TRAIN.0.NOZZLE.DIAMETER:0.4\n";
    file << ";EXTRUDER_TRAIN.0.NOZZLE.NAME:AA 0.4\n";
    file << ";BUILD_PLATE.INITIAL_TEMPERATURE:60\n";
    file << ";PRINT.TIME:5201\n";
    file << ";PRINT.SIZE.MIN.X:9\n";
    file << ";PRINT.SIZE.MIN.Y:6\n";
    file << ";PRINT.SIZE.MIN.Z:0.27\n";
    file << ";PRINT.SIZE.MAX.X:173.325\n";
    file << ";PRINT.SIZE.MAX.Y:164.325\n";
    file << ";PRINT.SIZE.MAX.Z:60.03\n";
    file << ";END_OF_HEADER\n";
    file << ";Generated with libArachne\n";
    file << "\n";
    file << "T0\n";
    file << "M82 ; absolute extrusion mode\n";
    file << "G92 E0\n";
    file << "M109 S215\n";
    file << "G0 F15000 X9 Y6 Z2\n";
    file << "G280\n";
    file << "G1 F1500 E-6.5\n";
    file << ";LAYER_COUNT:1\n";
    file << ";LAYER:0\n";
    file << "M107\n";
    file << "M204 S625\n";
    file << "M205 X6 Y6\n";
    file << "G0 X" << INT2MM(build_plate_middle.X) << " Y" << INT2MM(build_plate_middle.Y) << " Z" << INT2MM(layer_thickness) << " F1200 ; start location\n";
    file << "G0 E0 F1500 ; unretract\n";
    file << "M204 S4000\n";
    file << "M205 X25 Y25\n";
    file << "\n";
    file << "M214 K0.4 ; bueno linear advance\n";
    file << "M83 ;relative extrusion mode\n";
    file << "\n";
}

GcodeWriter::~GcodeWriter()
{
    
    file << "M214 K0.0\n";
    file << "M107\n";
    file.close();
}

void GcodeWriter::print(std::vector<std::vector<std::vector<ExtrusionJunction>>>& polygons_per_index, std::vector<std::vector<std::vector<ExtrusionJunction>>>& polylines_per_index, AABB aabb)
{
    reduction = aabb.getMiddle() - build_plate_middle;
    for (int inset_idx = 0; inset_idx < std::max(polygons_per_index.size(), polylines_per_index.size()); inset_idx++)
    {
        if (inset_idx < polygons_per_index.size())
        {
            for (std::vector<ExtrusionJunction>& polygon : polygons_per_index[inset_idx])
            {
                move(polygon.back().p);
                ExtrusionJunction* last = &polygon.back();
                for (ExtrusionJunction& junction : polygon)
                {
                    print(*last, junction);
                    last = &junction;
                }
            }
        }
        if (inset_idx < polylines_per_index.size())
        {
            for (std::vector<ExtrusionJunction>& polyline : polylines_per_index[inset_idx])
            {
                move(polyline.front().p);
                ExtrusionJunction* last = &polyline.front();
                for (coord_t junction_idx = 1; junction_idx < polyline.size(); junction_idx++)
                {
                    ExtrusionJunction& junction = polyline[junction_idx];
                    print(*last, junction);
                    last = &junction;
                }
            }
        }
    }
}

void GcodeWriter::move(Point p)
{
    p -= reduction;
    switch(type)
    {
        case type_P3:
            file << "G5 F" << travel_speed << " X" << INT2MM(p.X) << " Y" << INT2MM(p.Y) << " S0 E0\n";
            break;
        case type_UM3:
        default:
            file << "G0 X" << INT2MM(p.X) << " Y" << INT2MM(p.Y) << "\n";
            break;
    }
    cur_pos = p;
}

void GcodeWriter::print(ExtrusionJunction from, ExtrusionJunction to)
{
    from.p -= reduction;
    to.p -= reduction;

    assert(from.p == cur_pos);
    bool discretize = type != type_P3
        && std::abs(to.w - from.w) < 10;

    if (!discretize)
    {
        printSingleExtrusionMove(from, to);
    }
    else
    {
        Point vec = to.p - from.p;
        coord_t length = vSize(vec);
        coord_t segment_count = (length + discretization_size / 2) / discretization_size; // round to nearest
        ExtrusionJunction last = from;
        for (coord_t segment_idx = 0; segment_idx < segment_count; segment_idx++)
        {
            ExtrusionJunction here(from.p + vec * (segment_idx + 1) / segment_count, from.w + (to.w - from.w) * (segment_idx + 1) / segment_count, last.perimeter_index);
            printSingleExtrusionMove(last, here);
            last = here;
        }
    }
    
    cur_pos = to.p;
}

void GcodeWriter::printSingleExtrusionMove(ExtrusionJunction& from, ExtrusionJunction& to)
{
    switch(type)
    {
        case type_P3:
            file << "G5 F" << print_speed << " X" << INT2MM(to.p.X) << " Y" << INT2MM(to.p.Y)
                << " S" << getExtrusionFilamentMmPerMmMove(from.w) << " E" << getExtrusionFilamentMmPerMmMove(to.w) << "\n";
            break;
        case type_UM3:
        default:
            last_E += getExtrusionFilamentMmPerMmMove((from.w + to.w) / 2) * vSize(to.p - from.p);
            file << "G1 F" << print_speed << " X" << INT2MM(to.p.X) << " Y" << INT2MM(to.p.Y)
                << " E" << last_E << "\n";
            break;
    }
}

float GcodeWriter::getExtrusionFilamentMmPerMmMove(coord_t width)
{
    float filament_radius = 0.5 * filament_diameter;
    float volume_per_mm_filament = M_PI * filament_radius * filament_radius;
    float volume_per_mm_move = INT2MM(width) * INT2MM(layer_thickness);
    // v / m / (v / f) = f / m
    return volume_per_mm_move / volume_per_mm_filament;
}

} // namespace arachne
