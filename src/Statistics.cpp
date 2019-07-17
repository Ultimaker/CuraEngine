//Copyright (c) 2019 Ultimaker B.V.


#include "Statistics.h"

#include <sstream>
#include <fstream>

#include "utils/logoutput.h"

namespace arachne
{

void Statistics::analyse(Polygons& input, std::vector<std::vector<std::vector<ExtrusionJunction>>>& polygons_per_index, std::vector<std::vector<std::vector<ExtrusionJunction>>>& polylines_per_index, VoronoiQuadrangulation* vq)
{
    this->input = &input;
    this->vq = vq;
    this->polygons_per_index = &polygons_per_index;
    this->polylines_per_index = &polylines_per_index;

    generateAllSegments(polygons_per_index, polylines_per_index);

    
    for (coord_t segment_idx = 0; segment_idx < all_segments.size(); segment_idx++)
    {
        Segment s = all_segments[segment_idx];
        Polygons covered = s.s.toPolygons(false);
        area_covered = area_covered.unionPolygons(covered);
        Polygons extruded = s.toPolygons();
        overlaps.add(extruded);
    }
    
    overfills = overlaps.xorPolygons(area_covered);
    overfills = overfills.offset(-5);
    overfills = overfills.offset(10);
    overfills = overfills.offset(-5);

    double total_overfill_area = INT2MM2(overfills.area());
    logAlways("Total overfill area: %f mm²\n", total_overfill_area);
    std::vector<PolygonsPart> overfill_areas = overfills.splitIntoParts();
    logAlways("Average area: %f mm² over %d parts\n", total_overfill_area / overfill_areas.size(), overfill_areas.size());

    underfills = input.difference(area_covered);
    underfills = underfills.offset(5);
    underfills = underfills.offset(-10);
    underfills = underfills.offset(5);

    double total_underfill_area = INT2MM2(underfills.area());
    logAlways("Total underfill area: %f mm²\n", total_underfill_area);
    std::vector<PolygonsPart> underfill_areas = underfills.splitIntoParts();
    logAlways("Average area: %f mm² over %d parts\n", total_underfill_area / underfill_areas.size(), underfill_areas.size());

    logAlways("Total target area: %f mm²\n", INT2MM2(input.area()));

    // initialize paths
    for (Segment& segment : all_segments)
    {
        PolygonRef poly = paths.newPoly();
        poly.emplace_back(segment.s.from.p);
        poly.emplace_back(segment.s.to.p);
    }

}

void Statistics::generateAllSegments(std::vector<std::vector<std::vector<ExtrusionJunction>>>& polygons_per_index, std::vector<std::vector<std::vector<ExtrusionJunction>>>& polylines_per_index)
{
    for (std::vector<std::vector<ExtrusionJunction>>& polygons : polygons_per_index)
    {
        for (std::vector<ExtrusionJunction>& polygon : polygons)
        {
            ExtrusionJunction last = polygon.back();
            for (coord_t junction_idx = 0; junction_idx < polygon.size(); junction_idx++)
            {
                ExtrusionJunction& junction = polygon[junction_idx];
                ExtrusionSegment segment(last, junction, false);
                all_segments.emplace_back(segment, false);
                last = junction;
            }
        }
    }
    for (std::vector<std::vector<ExtrusionJunction>>& polylines : polylines_per_index)
    {
        for (std::vector<ExtrusionJunction>& polyline : polylines)
        {
            ExtrusionJunction last = polyline.front();
            for (coord_t junction_idx = 0; junction_idx < polyline.size(); junction_idx++)
            {
                ExtrusionJunction& junction = polyline[junction_idx];
                ExtrusionSegment segment(last, junction, false);
                all_segments.emplace_back(segment, junction_idx == polyline.size() - 1);
                last = junction;
            }
        }
    }
}

void Statistics::visualize()
{
    AABB aabb(*input);

    if (vq)
    {
        std::ostringstream ss;
        ss << "output/" << filename_base << "_after.svg";
        SVG svg(ss.str(), aabb);
        svg.writePolygons(*input, SVG::Color::GRAY, 2);
        vq->debugOutput(svg, false, false, true);
        svg.writePolygons(paths, SVG::Color::BLACK, 2);
        
        for (auto polys : *polylines_per_index)
        {
            for (auto poly : polys)
            {
                Point prev = poly.front().p;
                for (ExtrusionJunction& j : poly)
                {
                    svg.writeLine(prev, j.p, SVG::Color::RED, 2);
                    prev = j.p;
                }
            }
        }
        for (auto polylines : *polylines_per_index)
        {
            for (std::vector<ExtrusionJunction>& polyline : polylines)
            {
                svg.writePoint(polyline.front().p, true, 5, SVG::Color::GREEN);
                svg.writePoint(polyline.back().p, true, 5, SVG::Color::BLUE);
            }
        }
    }

    {
        std::ostringstream ss;
        ss << "output/" << filename_base << "_toolpaths.svg";
        SVG svg(ss.str(), aabb);
        svg.writeAreas(*input, SVG::Color::GRAY, SVG::Color::BLACK, 2);
        bool alternate = true;
        for (PolygonRef poly : overlaps)
        {
            svg.writeAreas(poly, alternate? SVG::Color::BLUE : SVG::Color::MAGENTA, SVG::Color::NONE);
            alternate = !alternate;
        }
        svg.writePolygons(paths, SVG::Color::BLACK, 2);
    }

    {
        std::ostringstream ss;
        ss << "output/" << filename_base << "_accuracy.svg";
        SVG svg(ss.str(), aabb);
        svg.writeAreas(*input, SVG::Color::GRAY, SVG::Color::BLACK, 3);
        svg.writeAreas(overfills, SVG::Color::RED, SVG::Color::NONE);
        svg.writeAreas(underfills, SVG::Color::BLUE, SVG::Color::NONE);
        svg.writePolygons(paths, SVG::Color::BLACK, 1);
    }

    if (false)
    {
        std::ofstream csv("output/segments.csv", std::ofstream::out | std::ofstream::trunc);
        csv << "from_x; from_y; from_width; to_x; to_y; to_width\n";
        for (const Segment& segment : all_segments)
            csv << segment.s.from.p.X << "; " << segment.s.from.p.Y << "; " << segment.s.from.w << "; " << segment.s.to.p.X << "; " << segment.s.to.p.Y << "; " << segment.s.to.w << '\n';
        csv.close();
    }

}
    

} // namespace arachne
