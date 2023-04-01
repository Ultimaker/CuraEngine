// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <sstream>

#include <spdlog/spdlog.h>

#include "utils/ExtrusionLine.h"
#include "utils/SVG.h"
#include "utils/floatpoint.h"
#include "utils/polygon.h"

namespace cura
{


std::string SVG::toString(Color color) const
{
    switch (color)
    {
    case SVG::Color::BLACK:
        return "black";
    case SVG::Color::WHITE:
        return "white";
    case SVG::Color::GRAY:
        return "gray";
    case SVG::Color::RED:
        return "red";
    case SVG::Color::BLUE:
        return "blue";
    case SVG::Color::GREEN:
        return "green";
    case SVG::Color::LIME:
        return "lime";
    case SVG::Color::ORANGE:
        return "orange";
    case SVG::Color::MAGENTA:
        return "magenta";
    case SVG::Color::YELLOW:
        return "yellow";
    case SVG::Color::NONE:
        return "none";
    default:
        return "black";
    }
}

std::string SVG::toString(const ColorObject& color) const
{
    if (color.is_enum)
        return toString(color.color);
    else
    {
        std::ostringstream ss;
        ss << "rgb(" << color.r << "," << color.g << "," << color.b << ")";
        return ss.str();
    }
}


SVG::SVG(std::string filename, AABB aabb, Point canvas_size, ColorObject background)
    : SVG(filename, aabb, std::min(double(canvas_size.X - canvas_size.X / 5 * 2) / (aabb.max.X - aabb.min.X), double(canvas_size.Y - canvas_size.Y / 5) / (aabb.max.Y - aabb.min.Y)), canvas_size, background)
{
}

SVG::SVG(std::string filename, AABB aabb, double scale, ColorObject background) : SVG(filename, aabb, scale, (aabb.max - aabb.min) * scale, background)
{
}

SVG::SVG(std::string filename, AABB aabb, double scale, Point canvas_size, ColorObject background) : aabb(aabb), aabb_size(aabb.max - aabb.min), canvas_size(canvas_size), scale(scale), background(background)
{
    output_is_html = strcmp(filename.c_str() + strlen(filename.c_str()) - 4, "html") == 0;
    out = fopen(filename.c_str(), "w");
    if (! out)
    {
        spdlog::error("The file %s could not be opened for writing.", filename);
    }
    if (output_is_html)
    {
        fprintf(out, "<!DOCTYPE html><html><body>\n");
    }
    else
    {
        fprintf(out, "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n");
    }
    fprintf(out, "<svg \n");
    fprintf(out, "   xmlns=\"http://www.w3.org/2000/svg\"\n");
    fprintf(out, "   xmlns:inkscape=\"http://www.inkscape.org/namespaces/inkscape\"\n");
    fprintf(out, "   height=\"%f\"\n", scale * (aabb.max.Y - aabb.min.Y));
    fprintf(out, "   width=\"%f\"\n", scale * (aabb.max.X - aabb.min.X));
    fprintf(out, "   version=\"1.1\">\n");
    fprintf(out, "  <g\n");
    fprintf(out, "    inkscape:groupmode=\"layer\"\n");
    fprintf(out, "    inkscape:label=\"layer%zu\"\n", layer_nr);
    fprintf(out, "    id=\"layer%zu\">\n", layer_nr);

    if (! background.is_enum || background.color != Color::NONE)
    {
        fprintf(out, "<rect width=\"100%%\" height=\"100%%\" fill=\"%s\"/>\n", toString(background).c_str());
    }
}

SVG::~SVG()
{
    fprintf(out, "  </g>\n");
    fprintf(out, "</svg>\n");
    if (output_is_html)
    {
        fprintf(out, "</body></html>");
    }
    fclose(out);
}

double SVG::getScale() const
{
    return scale;
}

void SVG::nextLayer()
{
    fprintf(out, "  </g>\n");
    layer_nr++;
    fprintf(out, "  <g\n");
    fprintf(out, "    inkscape:groupmode=\"layer\"\n");
    fprintf(out, "    inkscape:label=\"layer%zu\"\n", layer_nr);
    fprintf(out, "    id=\"layer%zu\">\n", layer_nr);
}

Point SVG::transform(const Point& p) const
{
    return Point((p.X - aabb.min.X) * scale, (p.Y - aabb.min.Y) * scale);
}

FPoint3 SVG::transformF(const Point& p) const
{
    return FPoint3((p.X - aabb.min.X) * scale, (p.Y - aabb.min.Y) * scale, 0.0);
}

void SVG::writeComment(const std::string& comment) const
{
    fprintf(out, "<!-- %s -->\n", comment.c_str());
}

void SVG::writeAreas(const Polygons& polygons, const ColorObject color, const ColorObject outline_color, const float stroke_width) const
{
    auto parts = polygons.splitIntoParts();
    for (auto part_it = parts.rbegin(); part_it != parts.rend(); ++part_it)
    {
        PolygonsPart& parts = *part_it;
        for (unsigned int j = 0; j < parts.size(); j++)
        {
            fprintf(out, "<polygon points=\"");
            for (Point& p : parts[j])
            {
                FPoint3 fp = transformF(p);
                fprintf(out, "%f,%f ", fp.x, fp.y);
            }
            if (j == 0)
                fprintf(out, "\" style=\"fill:%s;stroke:%s;stroke-width:%f\" />\n", toString(color).c_str(), toString(outline_color).c_str(), stroke_width);
            else
                fprintf(out, "\" style=\"fill:white;stroke:%s;stroke-width:%f\" />\n", toString(outline_color).c_str(), stroke_width);
        }
    }
}

void SVG::writeAreas(ConstPolygonRef polygon, const ColorObject color, const ColorObject outline_color, const float stroke_width) const
{
    fprintf(out, "<polygon fill=\"%s\" stroke=\"%s\" stroke-width=\"%f\" points=\"", toString(color).c_str(), toString(outline_color).c_str(), stroke_width); // The beginning of the polygon tag.
    for (const Point& point : polygon) // Add every point to the list of points.
    {
        FPoint3 transformed = transformF(point);
        fprintf(out, "%f,%f ", transformed.x, transformed.y);
    }
    fprintf(out, "\" />\n"); // The end of the polygon tag.
}

void SVG::writePoint(const Point& p, const bool write_coords, const float size, const ColorObject color) const
{
    FPoint3 pf = transformF(p);
    fprintf(out, "<circle cx=\"%f\" cy=\"%f\" r=\"%f\" stroke-width=\"0\" fill=\"%s\" />\n", pf.x, pf.y, size, toString(color).c_str());

    if (write_coords)
    {
        fprintf(out, "<text x=\"%f\" y=\"%f\" style=\"font-size: 10px;\" fill=\"black\">%lli,%lli</text>\n", pf.x, pf.y, p.X, p.Y);
    }
}

void SVG::writePoints(ConstPolygonRef poly, const bool write_coords, const float size, const ColorObject color) const
{
    for (const Point& p : poly)
    {
        writePoint(p, write_coords, size, color);
    }
}

void SVG::writePoints(const Polygons& polygons, const bool write_coords, const float size, const ColorObject color) const
{
    for (const ConstPolygonRef& poly : polygons)
    {
        writePoints(poly, write_coords, size, color);
    }
}

void SVG::writeLines(const std::vector<Point>& polyline, const ColorObject color) const
{
    if (polyline.size() <= 1) // Need at least 2 points.
    {
        return;
    }

    FPoint3 transformed = transformF(polyline[0]); // Element 0 must exist due to the check above.
    fprintf(out, "<path fill=\"none\" stroke=\"%s\" stroke-width=\"1\" d=\"M%f,%f", toString(color).c_str(), transformed.x, transformed.y); // Write the start of the path tag and the first endpoint.
    for (size_t point = 1; point < polyline.size(); point++)
    {
        transformed = transformF(polyline[point]);
        fprintf(out, "L%f,%f", transformed.x, transformed.y); // Write a line segment to the next point.
    }
    fprintf(out, "\" />\n"); // Write the end of the tag.
}

void SVG::writeLine(const Point& a, const Point& b, const ColorObject color, const float stroke_width) const
{
    FPoint3 fa = transformF(a);
    FPoint3 fb = transformF(b);
    fprintf(out, "<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" style=\"stroke:%s;stroke-width:%f\" />\n", fa.x, fa.y, fb.x, fb.y, toString(color).c_str(), stroke_width);
}

void SVG::writeArrow(const Point& a, const Point& b, const ColorObject color, const float stroke_width, const float head_size) const
{
    FPoint3 fa = transformF(a);
    FPoint3 fb = transformF(b);
    FPoint3 ab = fb - fa;
    FPoint3 normal = FPoint3(ab.y, -ab.x, 0.0).normalized();
    FPoint3 direction = ab.normalized();

    FPoint3 tip = fb + normal * head_size - direction * head_size;
    FPoint3 b_base = fb + normal * stroke_width - direction * stroke_width * 2.41;
    FPoint3 a_base = fa + normal * stroke_width;
    fprintf(out, "<polygon fill=\"%s\" points=\"%f,%f %f,%f %f,%f %f,%f %f,%f\" />", toString(color).c_str(), fa.x, fa.y, fb.x, fb.y, tip.x, tip.y, b_base.x, b_base.y, a_base.x, a_base.y);
}

void SVG::writeLineRGB(const Point& from, const Point& to, const int r, const int g, const int b, const float stroke_width) const
{
    FPoint3 fa = transformF(from);
    FPoint3 fb = transformF(to);
    fprintf(out, "<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" style=\"stroke:rgb(%i,%i,%i);stroke-width:%f\" />\n", fa.x, fa.y, fb.x, fb.y, r, g, b, stroke_width);
}

void SVG::writeDashedLine(const Point& a, const Point& b, ColorObject color) const
{
    FPoint3 fa = transformF(a);
    FPoint3 fb = transformF(b);
    fprintf(out, "<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" stroke=\"%s\" stroke-width=\"1\" stroke-dasharray=\"5,5\" />\n", fa.x, fa.y, fb.x, fb.y, toString(color).c_str());
}

void SVG::writeText(const Point& p, const std::string& txt, const ColorObject color, const float font_size) const
{
    FPoint3 pf = transformF(p);
    fprintf(out, "<text x=\"%f\" y=\"%f\" style=\"font-size: %fpx;\" fill=\"%s\">%s</text>\n", pf.x, pf.y, font_size, toString(color).c_str(), txt.c_str());
}

void SVG::writePolygons(const Polygons& polys, const ColorObject color, const float stroke_width) const
{
    for (ConstPolygonRef poly : polys)
    {
        writePolygon(poly, color, stroke_width);
    }
}

void SVG::writePolygon(ConstPolygonRef poly, const ColorObject color, const float stroke_width) const
{
    if (poly.size() == 0)
    {
        return;
    }
    int size = poly.size();
    Point p0 = poly.back();
    int i = 0;
    for (Point p1 : poly)
    {
        if (color.color == Color::RAINBOW)
        {
            int g = (i * 255 * 11 / size) % (255 * 2);
            if (g > 255)
            {
                g = 255 * 2 - g;
            }
            int b = (i * 255 * 5 / size) % (255 * 2);
            if (b > 255)
            {
                b = 255 * 2 - b;
            }
            writeLineRGB(p0, p1, i * 255 / size, g, b, stroke_width);
        }
        else
        {
            writeLine(p0, p1, color, stroke_width);
        }
        p0 = p1;
        i++;
    }
}


void SVG::writePolylines(const Polygons& polys, const ColorObject color, const float stroke_width) const
{
    for (ConstPolygonRef poly : polys)
    {
        writePolyline(poly, color, stroke_width);
    }
}

void SVG::writePolyline(ConstPolygonRef poly, const ColorObject color, const float stroke_width) const
{
    if (poly.size() == 0)
    {
        return;
    }
    int size = poly.size();
    Point p0 = poly[0];
    int i = 0;
    for (size_t p_idx = 1; p_idx < poly.size(); p_idx++)
    {
        Point p1 = poly[p_idx];
        if (color.color == Color::RAINBOW)
        {
            int g = (i * 255 * 11 / size) % (255 * 2);
            if (g > 255)
                g = 255 * 2 - g;
            int b = (i * 255 * 5 / size) % (255 * 2);
            if (b > 255)
                b = 255 * 2 - b;
            writeLineRGB(p0, p1, i * 255 / size, g, b, stroke_width);
        }
        else
        {
            writeLine(p0, p1, color, stroke_width);
        }
        p0 = p1;
        i++;
    }
}

void SVG::writePaths(const std::vector<VariableWidthLines>& paths, const ColorObject color, const float width_factor) const
{
    for (const VariableWidthLines& lines : paths)
    {
        writeLines(lines, color, width_factor);
    }
}

void SVG::writeLines(const VariableWidthLines& lines, const ColorObject color, const float width_factor) const
{
    for (const ExtrusionLine& line : lines)
    {
        writeLine(line, color, width_factor);
    }
}

void SVG::writeLine(const ExtrusionLine& line, const ColorObject color, const float width_factor) const
{
    constexpr float minimum_line_width = 10; // Always have some width, otherwise some lines become completely invisible.
    if (line.junctions.empty()) // Only draw lines that have at least 2 junctions, otherwise they are degenerate.
    {
        return;
    }
    ExtrusionJunction start_vertex = line.junctions[0];
    for (size_t index = 1; index < line.junctions.size(); ++index)
    {
        ExtrusionJunction end_vertex = line.junctions[index];

        // Compute the corners of the trapezoid for this variable-width line segment.
        const Point direction_vector = end_vertex.p - start_vertex.p;
        const Point direction_left = turn90CCW(direction_vector);
        const Point direction_right = -direction_left; // Opposite of left.
        const FPoint3 start_left = transformF(start_vertex.p + normal(direction_left, std::max(minimum_line_width, start_vertex.w * width_factor)));
        const FPoint3 start_right = transformF(start_vertex.p + normal(direction_right, std::max(minimum_line_width, start_vertex.w * width_factor)));
        const FPoint3 end_left = transformF(end_vertex.p + normal(direction_left, std::max(minimum_line_width, end_vertex.w * width_factor)));
        const FPoint3 end_right = transformF(end_vertex.p + normal(direction_right, std::max(minimum_line_width, end_vertex.w * width_factor)));

        fprintf(out, "<polygon fill=\"%s\" points=\"%f,%f %f,%f %f,%f %f,%f\" />\n", toString(color).c_str(), start_left.x, start_left.y, start_right.x, start_right.y, end_right.x, end_right.y, end_left.x, end_left.y);

        start_vertex = end_vertex; // For the next line segment.
    }
}

void SVG::writeCoordinateGrid(const coord_t grid_size, const Color color, const float stroke_width, const float font_size) const
{
    constexpr float dist_from_edge = 0.05; // As fraction of image width or height.
    const coord_t min_x = aabb.min.X - (aabb.min.X % grid_size);
    const coord_t min_y = aabb.min.Y - (aabb.min.Y % grid_size);

    for (coord_t x = min_x; x < aabb.max.X; x += grid_size)
    {
        writeLine(Point(x, aabb.min.Y), Point(x, aabb.max.Y), color, stroke_width);
        std::stringstream ss;
        ss << INT2MM(x);
        writeText(Point(x, aabb.min.Y + (aabb.max.Y - aabb.min.Y) * dist_from_edge), ss.str(), color, font_size);
    }
    for (coord_t y = min_y; y < aabb.max.Y; y += grid_size)
    {
        writeLine(Point(aabb.min.X, y), Point(aabb.max.Y, y), color, stroke_width);
        std::stringstream ss;
        ss << INT2MM(y);
        writeText(Point(aabb.min.X + (aabb.max.X - aabb.min.X) * dist_from_edge, y), ss.str(), color, font_size);
    }
}

} // namespace cura
