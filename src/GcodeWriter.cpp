//Copyright (c) 2019 Ultimaker B.V.


#include "GcodeWriter.h"
#include "pathOrderOptimizer.h"

namespace arachne
{
GcodeWriter::GcodeWriter(std::string filename, int type, coord_t layer_thickness, float print_speed, float travel_speed, float extrusion_multiplier)
: file(filename.c_str())
, type(type)
, layer_thickness(layer_thickness)
, print_speed(print_speed)
, travel_speed(travel_speed)
, extrusion_multiplier(extrusion_multiplier)
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
    file << "M204 S625; set acceleration\n";
    file << "M205 X6 Y6; set jerk\n";
    file << "G0 X" << INT2MM(build_plate_middle.X) << " Y" << INT2MM(build_plate_middle.Y) << " Z" << (INT2MM(layer_thickness) + 0.24) << " F1200 ; start location\n";
    file << "G0 E0 F1500 ; unretract\n";
    file << "\n";
    file << "M214 K2.0 ; bueno linear advance\n";
//     file << "M83 ;relative extrusion mode\n";
    file << "\n";
    cur_pos = build_plate_middle;
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
        if (inset_idx < polylines_per_index.size())
        {
            LineOrderOptimizer order_optimizer(cur_pos);
            Polygons recreated;
            for (std::vector<ExtrusionJunction>& polyline : polylines_per_index[inset_idx])
            {
                PolygonRef recreated_poly = recreated.newPoly();
                recreated_poly.add(polyline.front().p);
                recreated_poly.add(polyline.back().p);
            }
            order_optimizer.addPolygons(recreated);
            order_optimizer.optimize();
            for (int poly_idx : order_optimizer.polyOrder)
            {
                std::vector<ExtrusionJunction>& polyline = polylines_per_index[inset_idx][poly_idx];
                int start_idx = order_optimizer.polyStart[poly_idx];
                assert(start_idx < polyline.size());
                ExtrusionJunction* last = (start_idx == 0)? &polyline.front() : &polyline.back();
                move(last->p);
                for (int junction_nr = 1; junction_nr < polyline.size(); junction_nr++)
                {
                    int junction_idx = (start_idx == 0)? junction_nr : polyline.size() - junction_nr - 1;
                    ExtrusionJunction& junction = polyline[junction_idx];
                    print(*last, junction);
                    last = &junction;
                }
            }
        }
        if (inset_idx < polygons_per_index.size())
        {
            PathOrderOptimizer order_optimizer(cur_pos);
            Polygons recreated;
            for (std::vector<ExtrusionJunction>& polygon : polygons_per_index[inset_idx])
            {
                PolygonRef recreated_poly = recreated.newPoly();
                for (ExtrusionJunction& j : polygon)
                {
                    recreated_poly.add(j.p);
                }
            }
            order_optimizer.addPolygons(recreated);
            order_optimizer.optimize();
            for (int poly_idx : order_optimizer.polyOrder)
            {
                std::vector<ExtrusionJunction>& polygon = polygons_per_index[inset_idx][poly_idx];
                int start_idx = order_optimizer.polyStart[poly_idx];
                assert(start_idx < polygon.size());
                ExtrusionJunction* last = &polygon[start_idx];
                move(last->p);
                for (int junction_nr = 1; junction_nr < polygon.size() + 1; junction_nr++)
                {
                    ExtrusionJunction& junction = polygon[(start_idx + junction_nr) % polygon.size()];
                    print(*last, junction);
                    last = &junction;
                }
            }
        }
    }
}

void GcodeWriter::move(Point p)
{
    file << "\n";
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
        && std::abs(to.w - from.w) > 10;

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
            coord_t w = (from.w + to.w) / 2;
            float speed = print_speed * nozzle_size / w;
            last_E += getExtrusionFilamentMmPerMmMove(w) * vSize(to.p - from.p);
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
    return volume_per_mm_move / volume_per_mm_filament * extrusion_multiplier;
}

} // namespace arachne
