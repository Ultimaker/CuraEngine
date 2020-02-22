//Copyright (c) 2019 Ultimaker B.V.


#include "Statistics.h"

#include <sstream>
#include <fstream>

#include "utils/logoutput.h"
#include "utils/ToolpathVisualizer.h"

namespace arachne
{

void Statistics::analyse(std::vector<std::list<ExtrusionLine>>& polygons_per_index, std::vector<std::list<ExtrusionLine>>& polylines_per_index, SkeletalTrapezoidation* st)
{
    this->st = st;
    this->polygons_per_index = &polygons_per_index;
    this->polylines_per_index = &polylines_per_index;

    generateAllSegments(polygons_per_index, polylines_per_index);

    
    for (size_t segment_idx = 0; segment_idx < all_segments.size(); segment_idx++)
    {
        ExtrusionSegment s = all_segments[segment_idx];
        Polygons covered = s.toPolygons(false);
        area_covered.add(covered);
        Polygons extruded = s.toPolygons();
        overlaps.add(extruded);
    }
    
    area_covered = area_covered.execute(ClipperLib::pftNonZero);

    overfills = overlaps;
    for (PolygonRef poly : area_covered)
    {
        PolygonRef new_poly = overfills.newPoly();
        for (coord_t point_idx = poly.size() - 1; point_idx >= 0; --point_idx)
        {
            new_poly.add(poly[point_idx]);
        }
    }
    overfills.add(area_covered.difference(input));

    double_overfills = overfills;
    for (PolygonRef poly : area_covered)
    {
        PolygonRef new_poly = double_overfills.newPoly();
        for (coord_t point_idx = poly.size() - 1; point_idx >= 0; --point_idx)
        {
            new_poly.add(poly[point_idx]);
        }
    }
    overfills = overfills.execute(ClipperLib::pftPositive);
    overfills = overfills.intersection(area_covered);
    overfills = overfills.offset(-5);
    overfills = overfills.offset(10);
    overfills = overfills.offset(-5);

    double_overfills = double_overfills.execute(ClipperLib::pftPositive);
    double_overfills = double_overfills.offset(-5);
    double_overfills = double_overfills.offset(10);
    double_overfills = double_overfills.offset(-5);

    overfill_area = INT2MM2(overfills.area());
    double_overfill_area = INT2MM2(double_overfills.area());
    double total_overfill_area = overfill_area + double_overfill_area;
//     logAlways("Total overfill area: %f mm²\n", total_overfill_area);

    underfills = input.difference(area_covered);
    underfills = underfills.offset(5);
    underfills = underfills.offset(-10);
    underfills = underfills.offset(5);

    total_underfill_area = INT2MM2(underfills.area());
//     logAlways("Total underfill area: %f mm²\n", total_underfill_area);
//     std::vector<PolygonsPart> underfill_areas = underfills.splitIntoParts();
//     logAlways("Average area: %f mm² over %d parts\n", total_underfill_area / underfill_areas.size(), underfill_areas.size());

//     logAlways("Total target area: %f mm²\n", total_target_area);

    // initialize paths
    for (ExtrusionSegment& segment : all_segments)
    {
        PolygonRef poly = paths.newPoly();
        poly.emplace_back(segment.from.p);
        poly.emplace_back(segment.to.p);
    }

}

void Statistics::saveResultsCSV()
{
    if ( ! all_segments.empty())
    {
        std::ostringstream ss;
        ss << "output/" << output_prefix << "_" << test_type << "_segments.csv";
        std::ofstream csv(ss.str(), std::ofstream::out | std::ofstream::trunc);
        csv << "from_x,from_y,from_width,to_x,to_y,to_width,filename_base,output_prefix,inset_index\n";
        for (const ExtrusionSegment& segment : all_segments)
        {
            if (segment.from.perimeter_index != segment.to.perimeter_index)
                std::cerr << "Inset index doesn't correspond!\n";
            csv << segment.from.p.X << "," << segment.from.p.Y << "," << segment.from.w << ","
                << segment.to.p.X << "," << segment.to.p.Y << "," << segment.to.w << ","
                << test_type << "," << output_prefix << ","
                << segment.from.perimeter_index << '\n';
        }
        csv.close();
    }
    {
        coord_t vert_count = 0;
        for (ConstPolygonRef poly : input)
            for (const Point& p : poly)
                vert_count++;
        std::ostringstream ss;
        ss << "output/" << output_prefix << "_" << test_type << "_results.csv";
        std::ofstream csv(ss.str(), std::ofstream::out | std::ofstream::trunc);
        csv << "processing_time,overfill_area,double_overfill_area,total_underfill_area,total_target_area,total_target_area_length,vert_count,test_type,output_prefix\n";
        csv << processing_time << "," << overfill_area << "," << double_overfill_area << "," << total_underfill_area << ","
            << total_target_area << "," << total_target_area_length << "," << vert_count << ","
            << test_type << "," << output_prefix << '\n';
        csv.close();
    }
}


void Statistics::savePrintTimeCSV(Duration print_time)
{
    std::cerr << "Print time: " << print_time << "\n";

    std::ostringstream ss;
    ss << "output/" << output_prefix << "_" << test_type << "_printtime.csv";
    std::ofstream csv(ss.str(), std::ofstream::out | std::ofstream::trunc);
    csv << test_type << "," << output_prefix << "," << float(print_time) << "," << print_time << '\n';
}

void Statistics::generateAllSegments(std::vector<std::list<ExtrusionLine>>& polygons_per_index, std::vector<std::list<ExtrusionLine>>& polylines_per_index)
{
    for (std::list<ExtrusionLine>& polygons : polygons_per_index)
    {
        for (ExtrusionLine& polygon : polygons)
        {
            auto last_it = --polygon.junctions.end();
            for (auto junction_it = polygon.junctions.begin(); junction_it != polygon.junctions.end(); ++junction_it)
            {
                ExtrusionJunction& junction = *junction_it;
                ExtrusionSegment segment(*last_it, junction, false, true);
                all_segments.emplace_back(segment);
                last_it = junction_it;
            }
        }
    }
    for (std::list<ExtrusionLine>& polylines : polylines_per_index)
    {
        for (ExtrusionLine& polyline : polylines)
        {
            auto last_it = polyline.junctions.begin();
            for (auto junction_it = ++polyline.junctions.begin(); junction_it != polyline.junctions.end(); ++junction_it)
            {
                ExtrusionJunction& junction = *junction_it;
                ExtrusionSegment segment(*last_it, junction, false, junction_it != --polyline.junctions.end());
                all_segments.emplace_back(segment);
                last_it = junction_it;
            }
        }
    }
}

void Statistics::visualize(coord_t nozzle_size, bool output_st, bool output_toolpaths, bool output_widths, bool include_legend, bool visualize_accuracy, bool exaggerate_widths, bool rounded_visualization)
{
    AABB aabb(input);

#ifdef DEBUG
    if (output_st && st)
    {
        std::ostringstream ss;
        ss << "output/" << output_prefix << "_" << test_type << "_after.svg";
        SVG svg(ss.str(), aabb);
        st->debugOutput(svg, false, false, true);
        svg.writePolylines(paths, SVG::Color::BLACK, 2);
        
        if (false)
        for (auto polys : *polylines_per_index)
        {
            for (auto poly : polys)
            {
                Point prev = poly.junctions.front().p;
                for (ExtrusionJunction& j : poly.junctions)
                {
                    svg.writeLine(prev, j.p, SVG::Color::RED, 2);
                    prev = j.p;
                }
            }
        }
        for (auto polylines : *polylines_per_index)
        {
            for (ExtrusionLine& polyline : polylines)
            {
                svg.writePoint(polyline.junctions.front().p, false, 2, SVG::Color::GREEN);
                svg.writePoint(polyline.junctions.back().p, false, 2, SVG::Color::BLUE);
            }
        }
    }
#endif // DEBUG

    if (output_toolpaths)
    {
        std::ostringstream ss;
        ss << "output/" << output_prefix << "_" << test_type << "_toolpaths.svg";
        SVG svg(ss.str(), aabb);
        ToolpathVisualizer viz(svg);
        viz.outline(input);
        bool alternate = true;
        for (PolygonRef poly : overlaps)
        {
            svg.writeAreas(poly, alternate? SVG::Color::BLUE : SVG::Color::MAGENTA, SVG::Color::NONE);
            alternate = !alternate;
        }
        svg.writePolylines(paths, SVG::Color::BLACK, 2);
    }

    if (visualize_accuracy)
    {
        std::ostringstream ss;
        ss << "output/" << output_prefix << "_" << test_type << "_accuracy.svg";
        SVG svg(ss.str(), aabb);
        ToolpathVisualizer viz(svg);
        viz.outline(input);
        viz.toolpaths(all_segments, rounded_visualization);
        viz.underfill(underfills);
        viz.overfill(overfills, double_overfills);
    }

    if (output_widths)
    {
        std::ostringstream ss;
        ss << "output/" << output_prefix << "_" << test_type << "_widths.svg";
        SVG svg(ss.str(), aabb);
        ToolpathVisualizer viz(svg);
        viz.outline(input);

        coord_t max_dev = nozzle_size / 2;
        coord_t min_w = 30;

        if (include_legend)
        {
            viz.width_legend(input, nozzle_size, max_dev, min_w, rounded_visualization);
        }

        viz.widths(all_segments, nozzle_size, max_dev, min_w, rounded_visualization);
    }
}


} // namespace arachne
