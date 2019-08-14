//Copyright (c) 2019 Ultimaker B.V.


#include "GcodeWriter.h"
#include "pathOrderOptimizer.h"
#include "OrderOptimizer.h"

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
    file << ";BUILD_PLATE.INITIAL_TEMPERATURE:20\n";
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
    file << "G0 F" << travel_speed << " X" << INT2MM(build_plate_middle.X / 2) << " Y" << INT2MM(build_plate_middle.Y / 2) << " Z" << (INT2MM(layer_thickness) + 0.27) << " ; start location\n";
    is_unretracted = true;
    file << "\n";
    file << "M214 K1.0 ; bueno linear advance\n";
//     file << "M83 ;relative extrusion mode\n";
    file << "\n";
    cur_pos = build_plate_middle;
}

GcodeWriter::~GcodeWriter()
{
    
    file << "G0 F" << travel_speed << " X" << 20 << " Y" << 20 << " Z" << (INT2MM(layer_thickness) + 0.18) << " ; start location\n";
//     file << "M214 K0.0\n";
    file << "M107\n";
    file.close();
}

void GcodeWriter::printBrim(AABB aabb, coord_t count, coord_t w, coord_t dist)
{
    std::vector<std::list<ExtrusionLine>> polygons_per_index;
    std::vector<std::list<ExtrusionLine>> polylines_per_index;
    polygons_per_index.resize(1);
    std::list<ExtrusionLine>& polygons = polygons_per_index[0];

    Polygons prev;
    prev.add(aabb.toPolygon());
    prev = prev.offset(dist * 2);
    
    for (int i = 0; i < count; i++)
    {
        Polygons skuurt = prev.offset(dist, ClipperLib::jtRound);
        for (PolygonRef poly : skuurt)
        {
            polygons.emplace_back(0, true);
            ExtrusionLine& polygon = polygons.back();
            for (Point p : poly)
            {
                polygon.junctions.emplace_back(p, w, 0);
            }
        }
        prev = skuurt;
    }
    print(polygons_per_index, polylines_per_index, aabb);
}

void GcodeWriter::print(std::vector<std::list<ExtrusionLine>>& polygons_per_index, std::vector<std::list<ExtrusionLine>>& polylines_per_index, AABB aabb, bool ordered)
{
    if (ordered)
    {
        printOrdered(polygons_per_index, polylines_per_index, aabb);
    }
    else
    {
        printUnordered(polygons_per_index, polylines_per_index, aabb);
    }
}

void GcodeWriter::printOrdered(std::vector<std::list<ExtrusionLine>>& polygons_per_index, std::vector<std::list<ExtrusionLine>>& polylines_per_index, AABB aabb)
{
    reduction = aabb.getMiddle() - build_plate_middle;

    std::vector<std::vector<ExtrusionLine>> polygons_per_index_vector;
    polygons_per_index_vector.resize(polygons_per_index.size());
    for (coord_t inset_idx = 0; inset_idx < polygons_per_index.size(); inset_idx++)
        polygons_per_index_vector[inset_idx].insert(polygons_per_index_vector[inset_idx].end(), polygons_per_index[inset_idx].begin(), polygons_per_index[inset_idx].end());

    std::vector<std::vector<ExtrusionLine>> polylines_per_index_vector;
    polylines_per_index_vector.resize(polylines_per_index.size());
    for (coord_t inset_idx = 0; inset_idx < polylines_per_index.size(); inset_idx++)
        polylines_per_index_vector[inset_idx].insert(polylines_per_index_vector[inset_idx].end(), polylines_per_index[inset_idx].begin(), polylines_per_index[inset_idx].end());

    for (int inset_idx = 0; inset_idx < std::max(polygons_per_index_vector.size(), polylines_per_index_vector.size()); inset_idx++)
    {
        if (inset_idx < polylines_per_index_vector.size())
        {
            LineOrderOptimizer order_optimizer(cur_pos);
            Polygons recreated;
            for (ExtrusionLine& polyline : polylines_per_index_vector[inset_idx])
            {
                PolygonRef recreated_poly = recreated.newPoly();
                recreated_poly.add(polyline.junctions.front().p);
                recreated_poly.add(polyline.junctions.back().p);
            }
            order_optimizer.addPolygons(recreated);
            order_optimizer.optimize();
            for (int poly_idx : order_optimizer.polyOrder)
            {
                ExtrusionLine& polyline = polylines_per_index_vector[inset_idx][poly_idx];
                int start_idx = order_optimizer.polyStart[poly_idx];
                assert(start_idx < polyline.junctions.size());
                if (start_idx == 0)
                {
                    auto last = polyline.junctions.begin();
                    move(last->p);
                    for (auto junction_it = ++polyline.junctions.begin(); junction_it != polyline.junctions.end(); ++junction_it)
                    {
                        ExtrusionJunction& junction = *junction_it;
                        print(*last, junction);
                        last = junction_it;
                    }
                }
                else
                {
                    auto last = polyline.junctions.rbegin();
                    move(last->p);
                    for (auto junction_it = ++polyline.junctions.rbegin(); junction_it != polyline.junctions.rend(); ++junction_it)
                    {
                        ExtrusionJunction& junction = *junction_it;
                        print(*last, junction);
                        last = junction_it;
                    }
                }
            }
        }
        if (inset_idx < polygons_per_index_vector.size())
        {
            PathOrderOptimizer order_optimizer(cur_pos);
            Polygons recreated;
            for (ExtrusionLine& polygon : polygons_per_index_vector[inset_idx])
            {
                PolygonRef recreated_poly = recreated.newPoly();
                for (ExtrusionJunction& j : polygon.junctions)
                {
                    recreated_poly.add(j.p);
                }
            }
            order_optimizer.addPolygons(recreated);
            order_optimizer.optimize();
            for (int poly_idx : order_optimizer.polyOrder)
            {
                ExtrusionLine& polygon = polygons_per_index_vector[inset_idx][poly_idx];
                int start_idx = order_optimizer.polyStart[poly_idx];
                assert(start_idx < polygon.junctions.size());
                auto first = polygon.junctions.begin();
                std::advance(first, start_idx);
                move(first->p);
                auto prev = first;
                
                auto second = first; second++;
                for (auto junction_it = second; ; ++junction_it)
                {
                    if (junction_it == polygon.junctions.end()) junction_it = polygon.junctions.begin();
                    ExtrusionJunction& junction = *junction_it;
                    print(*prev, junction);
                    prev = junction_it;
                    if (junction_it == first) break;
                }
            }
        }
    }
}


void GcodeWriter::printUnordered(std::vector<std::list<ExtrusionLine>>& polygons_per_index, std::vector<std::list<ExtrusionLine>>& polylines_per_index, AABB aabb)
{
    reduction = aabb.getMiddle() - build_plate_middle;

    struct Path
    {
        std::vector<ExtrusionJunction> junctions;
        bool is_closed;
        Path(bool is_closed)
        : is_closed(is_closed)
        {}
    };
    std::vector<Path> paths;
    Polygons recreated;
    for (bool is_closed : {true, false})
    {
        auto polys_per_index = is_closed? polygons_per_index : polylines_per_index;
        for (auto polys : polys_per_index)
            for (auto poly : polys)
            {
                paths.emplace_back(is_closed);
                Path& path = paths.back();
                path.junctions.insert(path.junctions.begin(), poly.junctions.begin(), poly.junctions.end());

                recreated.emplace_back();
                PolygonRef recreated_poly = recreated.back();
                for (auto j : poly.junctions)
                    recreated_poly.add(j.p);
            }
    }
    OrderOptimizer order_optimizer(cur_pos);
    for (coord_t path_idx = 0; path_idx < paths.size(); path_idx++)
    {
        order_optimizer.addPoly(recreated[path_idx], paths[path_idx].is_closed);
    }
    order_optimizer.optimize();


    for (int poly_idx : order_optimizer.polyOrder)
    {
        Path& poly = paths[poly_idx];
        int start_idx = order_optimizer.polyStart[poly_idx];
        assert(start_idx < poly.junctions.size());
        ExtrusionJunction* prev = &poly.junctions[start_idx];
        move(prev->p);

        for (size_t pos = 1; pos < poly.junctions.size(); pos++)
        {
            size_t idx;
            if (poly.is_closed) idx = (start_idx + pos) % poly.junctions.size();
            else idx = (start_idx == 0)? pos : poly.junctions.size() - 1 - pos;
            ExtrusionJunction& here = poly.junctions[idx];
            print(*prev, here);
            prev = &here;
        }
        if (poly.is_closed)
        {
            print(*prev, poly.junctions[start_idx]);
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
    if (is_unretracted)
    {
        file << "G0 E0 F1500 ; unretract\n";
        is_unretracted = false;
    }
    from.p -= reduction;
    to.p -= reduction;

    assert(from.p == cur_pos);
    bool discretize = type != type_P3
        && std::abs(to.w - from.w) > 10;

    if (from.p == to.p)
    {
        return;
    }
    
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
