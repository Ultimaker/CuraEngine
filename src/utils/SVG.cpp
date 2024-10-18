// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/SVG.h"

#include <sstream>

#include <spdlog/spdlog.h>

#include "geometry/Polygon.h"
#include "geometry/SingleShape.h"
#include "utils/ExtrusionLine.h"
#include "utils/Point3D.h"

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
    if (color.is_enum_)
        return toString(color.color_);
    else
    {
        std::ostringstream ss;
        ss << "rgb(" << color.r_ << "," << color.g_ << "," << color.b_ << ")";
        return ss.str();
    }
}

void SVG::handleFlush(const bool flush) const
{
    if (flush)
    {
        fflush(out_);
    }
}


SVG::SVG(std::string filename, AABB aabb, Point2LL canvas_size, ColorObject background)
    : SVG(filename,
          aabb,
          std::min(
              static_cast<double>(canvas_size.X - canvas_size.X / 5 * 2) / static_cast<double>(aabb.max_.X - aabb.min_.X),
              static_cast<double>(canvas_size.Y - canvas_size.Y / 5) / static_cast<double>(aabb.max_.Y - aabb.min_.Y)),
          canvas_size,
          background)
{
}

SVG::SVG(std::string filename, AABB aabb, double scale, ColorObject background)
    : SVG(filename, aabb, scale, (aabb.max_ - aabb.min_) * scale, background)
{
}

SVG::SVG(std::string filename, AABB aabb, double scale, Point2LL canvas_size, ColorObject background)
    : aabb_(aabb)
    , aabb_size_(aabb.max_ - aabb.min_)
    , canvas_size_(canvas_size)
    , scale_(scale)
    , background_(background)
{
    output_is_html_ = strcmp(filename.c_str() + strlen(filename.c_str()) - 4, "html") == 0;
    out_ = fopen(filename.c_str(), "w");
    if (! out_)
    {
        spdlog::error("The file {} could not be opened for writing.", filename);
    }
    if (output_is_html_)
    {
        fprintf(out_, "<!DOCTYPE html><html><body>\n");
    }
    else
    {
        fprintf(out_, "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n");
    }
    fprintf(out_, "<svg \n");
    fprintf(out_, "   xmlns=\"http://www.w3.org/2000/svg\"\n");
    fprintf(out_, "   xmlns:inkscape=\"http://www.inkscape.org/namespaces/inkscape\"\n");
    fprintf(out_, "   height=\"%f\"\n", scale_ * static_cast<double>(aabb_.max_.Y - aabb_.min_.Y));
    fprintf(out_, "   width=\"%f\"\n", scale_ * static_cast<double>(aabb_.max_.X - aabb_.min_.X));
    fprintf(out_, "   version=\"1.1\">\n");
    fprintf(out_, "  <g\n");
    fprintf(out_, "    inkscape:groupmode=\"layer\"\n");
    fprintf(out_, "    inkscape:label=\"layer%zu\"\n", layer_nr_);
    fprintf(out_, "    id=\"layer%zu\">\n", layer_nr_);

    if (! background_.is_enum_ || background_.color_ != Color::NONE)
    {
        fprintf(out_, "<rect width=\"100%%\" height=\"100%%\" fill=\"%s\"/>\n", toString(background_).c_str());
    }
}

SVG::~SVG()
{
    fprintf(out_, "  </g>\n");
    fprintf(out_, "</svg>\n");
    if (output_is_html_)
    {
        fprintf(out_, "</body></html>");
    }
    fclose(out_);
}

double SVG::getScale() const
{
    return scale_;
}

void SVG::nextLayer()
{
    fprintf(out_, "  </g>\n");
    layer_nr_++;
    fprintf(out_, "  <g\n");
    fprintf(out_, "    inkscape:groupmode=\"layer\"\n");
    fprintf(out_, "    inkscape:label=\"layer%zu\"\n", layer_nr_);
    fprintf(out_, "    id=\"layer%zu\">\n", layer_nr_);
}

Point2LL SVG::transform(const Point2LL& p) const
{
    return Point2LL(std::llrint(static_cast<double>(p.X - aabb_.min_.X) * scale_), std::llrint(static_cast<double>(p.Y - aabb_.min_.Y) * scale_));
}

Point3D SVG::transformF(const Point2LL& p) const
{
    return Point3D(static_cast<double>(p.X - aabb_.min_.X) * scale_, static_cast<double>(p.Y - aabb_.min_.Y) * scale_, 0.0);
}

void SVG::writeComment(const std::string& comment) const
{
    fprintf(out_, "<!-- %s -->\n", comment.c_str());
}

void SVG::writeAreas(const Shape& polygons, const ColorObject color, const ColorObject outline_color, const double stroke_width) const
{
    std::vector<SingleShape> parts = polygons.splitIntoParts();
    for (auto part_it = parts.rbegin(); part_it != parts.rend(); ++part_it)
    {
        SingleShape& part = *part_it;
        for (size_t j = 0; j < part.size(); j++)
        {
            fprintf(out_, "<polygon points=\"");
            for (Point2LL& p : part[j])
            {
                Point3D fp = transformF(p);
                fprintf(out_, "%f,%f ", static_cast<double>(fp.x_), static_cast<double>(fp.y_));
            }
            if (j == 0)
                fprintf(out_, "\" style=\"fill:%s;stroke:%s;stroke-width:%f\" />\n", toString(color).c_str(), toString(outline_color).c_str(), static_cast<double>(stroke_width));
            else
                fprintf(out_, "\" style=\"fill:white;stroke:%s;stroke-width:%f\" />\n", toString(outline_color).c_str(), static_cast<double>(stroke_width));
        }
    }
}

void SVG::writeAreas(const Polygon& polygon, const ColorObject color, const ColorObject outline_color, const double stroke_width) const
{
    fprintf(
        out_,
        "<polygon fill=\"%s\" stroke=\"%s\" stroke-width=\"%f\" points=\"",
        toString(color).c_str(),
        toString(outline_color).c_str(),
        static_cast<double>(stroke_width)); // The beginning of the polygon tag.
    for (const Point2LL& point : polygon) // Add every point to the list of points.
    {
        Point3D transformed = transformF(point);
        fprintf(out_, "%f,%f ", static_cast<double>(transformed.x_), static_cast<double>(transformed.y_));
    }
    fprintf(out_, "\" />\n"); // The end of the polygon tag.
}

void SVG::writePoint(const Point2LL& p, const bool write_coords, const double size, const ColorObject color) const
{
    Point3D pf = transformF(p);
    fprintf(
        out_,
        "<circle cx=\"%f\" cy=\"%f\" r=\"%f\" stroke-width=\"0\" fill=\"%s\" />\n",
        static_cast<double>(pf.x_),
        static_cast<double>(pf.y_),
        static_cast<double>(size),
        toString(color).c_str());

    if (write_coords)
    {
        fprintf(out_, "<text x=\"%f\" y=\"%f\" style=\"font-size: 10px;\" fill=\"black\">%lli,%lli</text>\n", static_cast<double>(pf.x_), static_cast<double>(pf.y_), p.X, p.Y);
    }
}

void SVG::writePoints(const Polygon& poly, const bool write_coords, const double size, const ColorObject color) const
{
    for (const Point2LL& p : poly)
    {
        writePoint(p, write_coords, size, color);
    }
}

void SVG::writePoints(const Shape& polygons, const bool write_coords, const double size, const ColorObject color) const
{
    for (const Polygon& poly : polygons)
    {
        writePoints(poly, write_coords, size, color);
    }
}

void SVG::writeLines(const std::vector<Point2LL>& polyline, const ColorObject color) const
{
    if (polyline.size() <= 1) // Need at least 2 points.
    {
        return;
    }

    Point3D transformed = transformF(polyline[0]); // Element 0 must exist due to the check above.
    fprintf(
        out_,
        "<path fill=\"none\" stroke=\"%s\" stroke-width=\"1\" d=\"M%f,%f",
        toString(color).c_str(),
        static_cast<double>(transformed.x_),
        static_cast<double>(transformed.y_)); // Write the start of the path tag and the first endpoint.
    for (size_t point = 1; point < polyline.size(); point++)
    {
        transformed = transformF(polyline[point]);
        fprintf(out_, "L%f,%f", static_cast<double>(transformed.x_), static_cast<double>(transformed.y_)); // Write a line segment to the next point.
    }
    fprintf(out_, "\" />\n"); // Write the end of the tag.
}

void SVG::writeLine(const Point2LL& a, const Point2LL& b, const ColorObject color, const double stroke_width, const bool flush) const
{
    Point3D fa = transformF(a);
    Point3D fb = transformF(b);
    fprintf(
        out_,
        "<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" style=\"stroke:%s;stroke-width:%f\" />\n",
        static_cast<double>(fa.x_),
        static_cast<double>(fa.y_),
        static_cast<double>(fb.x_),
        static_cast<double>(fb.y_),
        toString(color).c_str(),
        static_cast<double>(stroke_width));

    handleFlush(flush);
}

void SVG::writeArrow(const Point2LL& a, const Point2LL& b, const ColorObject color, const double stroke_width, const double head_size) const
{
    Point3D fa = transformF(a);
    Point3D fb = transformF(b);
    Point3D ab = fb - fa;
    Point3D normal = Point3D(ab.y_, -ab.x_, 0.0).normalized();
    Point3D direction = ab.normalized();

    Point3D tip = fb + normal * head_size - direction * head_size;
    Point3D b_base = fb + normal * stroke_width - direction * stroke_width * 2.41f;
    Point3D a_base = fa + normal * stroke_width;
    fprintf(
        out_,
        "<polygon fill=\"%s\" points=\"%f,%f %f,%f %f,%f %f,%f %f,%f\" />",
        toString(color).c_str(),
        static_cast<double>(fa.x_),
        static_cast<double>(fa.y_),
        static_cast<double>(fb.x_),
        static_cast<double>(fb.y_),
        static_cast<double>(tip.x_),
        static_cast<double>(tip.y_),
        static_cast<double>(b_base.x_),
        static_cast<double>(b_base.y_),
        static_cast<double>(a_base.x_),
        static_cast<double>(a_base.y_));
}

void SVG::writeLineRGB(const Point2LL& from, const Point2LL& to, const int r, const int g, const int b, const double stroke_width) const
{
    Point3D fa = transformF(from);
    Point3D fb = transformF(to);
    fprintf(
        out_,
        "<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" style=\"stroke:rgb(%i,%i,%i);stroke-width:%f\" />\n",
        static_cast<double>(fa.x_),
        static_cast<double>(fa.y_),
        static_cast<double>(fb.x_),
        static_cast<double>(fb.y_),
        r,
        g,
        b,
        static_cast<double>(stroke_width));
}

void SVG::writeDashedLine(const Point2LL& a, const Point2LL& b, ColorObject color) const
{
    Point3D fa = transformF(a);
    Point3D fb = transformF(b);
    fprintf(
        out_,
        "<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" stroke=\"%s\" stroke-width=\"1\" stroke-dasharray=\"5,5\" />\n",
        static_cast<double>(fa.x_),
        static_cast<double>(fa.y_),
        static_cast<double>(fb.x_),
        static_cast<double>(fb.y_),
        toString(color).c_str());
}

void SVG::writeText(const Point2LL& p, const std::string& txt, const ColorObject color, const double font_size) const
{
    Point3D pf = transformF(p);
    fprintf(
        out_,
        "<text x=\"%f\" y=\"%f\" style=\"font-size: %fpx;\" fill=\"%s\">%s</text>\n",
        static_cast<double>(pf.x_),
        static_cast<double>(pf.y_),
        static_cast<double>(font_size),
        toString(color).c_str(),
        txt.c_str());
}

void SVG::writePolygons(const Shape& polys, const ColorObject color, const double stroke_width, const bool flush) const
{
    for (const Polygon& poly : polys)
    {
        writePolygon(poly, color, stroke_width, false);
    }

    handleFlush(flush);
}

void SVG::writePolygon(const Polygon poly, const ColorObject color, const double stroke_width, const bool flush) const
{
    if (poly.size() == 0)
    {
        return;
    }
    int size = static_cast<int>(poly.size());
    Point2LL p0 = poly.back();
    int i = 0;
    for (Point2LL p1 : poly)
    {
        if (color.color_ == Color::RAINBOW)
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

    handleFlush(flush);
}


void SVG::writePolylines(const Shape& polys, const ColorObject color, const double stroke_width, const bool flush) const
{
    for (const Polygon& poly : polys)
    {
        writePolyline(poly, color, stroke_width, false);
    }

    handleFlush(flush);
}

void SVG::writePolyline(const Polygon& poly, const ColorObject color, const double stroke_width) const
{
    if (poly.size() == 0)
    {
        return;
    }
    const int size = static_cast<int>(poly.size());
    Point2LL p0 = poly[0];
    int i = 0;
    for (size_t p_idx = 1; p_idx < poly.size(); p_idx++)
    {
        Point2LL p1 = poly[p_idx];
        if (color.color_ == Color::RAINBOW)
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

void SVG::writePolyline(const Polyline& poly, const ColorObject color, const double stroke_width, const bool flush) const
{
    for (auto iterator = poly.beginSegments(); iterator != poly.endSegments(); ++iterator)
    {
        writeLine((*iterator).start, (*iterator).end, color, stroke_width, false);
    }

    handleFlush(flush);
}

void SVG::writePaths(const std::vector<VariableWidthLines>& paths, const ColorObject color, const double width_factor) const
{
    for (const VariableWidthLines& lines : paths)
    {
        writeLines(lines, color, width_factor);
    }
}

void SVG::writeLines(const VariableWidthLines& lines, const ColorObject color, const double width_factor) const
{
    for (const ExtrusionLine& line : lines)
    {
        writeLine(line, color, width_factor);
    }
}

void SVG::writeLine(const ExtrusionLine& line, const ColorObject color, const double width_factor, const bool flush) const
{
    constexpr double minimum_line_width = 10; // Always have some width, otherwise some lines become completely invisible.
    if (line.junctions_.empty()) // Only draw lines that have at least 2 junctions, otherwise they are degenerate.
    {
        return;
    }
    ExtrusionJunction start_vertex = line.junctions_[0];
    for (size_t index = 1; index < line.junctions_.size(); ++index)
    {
        ExtrusionJunction end_vertex = line.junctions_[index];

        // Compute the corners of the trapezoid for this variable-width line segment.
        const Point2LL direction_vector = end_vertex.p_ - start_vertex.p_;
        const Point2LL direction_left = turn90CCW(direction_vector);
        const Point2LL direction_right = -direction_left; // Opposite of left.
        const Point3D start_left
            = transformF(start_vertex.p_ + normal(direction_left, std::llrint(std::max(minimum_line_width, static_cast<double>(start_vertex.w_) * width_factor))));
        const Point3D start_right
            = transformF(start_vertex.p_ + normal(direction_right, std::llrint(std::max(minimum_line_width, static_cast<double>(start_vertex.w_) * width_factor))));
        const Point3D end_left = transformF(end_vertex.p_ + normal(direction_left, std::llrint(std::max(minimum_line_width, static_cast<double>(end_vertex.w_) * width_factor))));
        const Point3D end_right = transformF(end_vertex.p_ + normal(direction_right, std::llrint(std::max(minimum_line_width, static_cast<double>(end_vertex.w_) * width_factor))));

        fprintf(
            out_,
            "<polygon fill=\"%s\" points=\"%f,%f %f,%f %f,%f %f,%f\" />\n",
            toString(color).c_str(),
            static_cast<double>(start_left.x_),
            static_cast<double>(start_left.y_),
            static_cast<double>(start_right.x_),
            static_cast<double>(start_right.y_),
            static_cast<double>(end_right.x_),
            static_cast<double>(end_right.y_),
            static_cast<double>(end_left.x_),
            static_cast<double>(end_left.y_));

        start_vertex = end_vertex; // For the next line segment.
    }

    if (flush)
    {
        fflush(out_);
    }
}

void SVG::writeCoordinateGrid(const coord_t grid_size, const Color color, const double stroke_width, const double font_size) const
{
    constexpr double dist_from_edge = 0.05; // As fraction of image width or height.
    const coord_t min_x = aabb_.min_.X - (aabb_.min_.X % grid_size);
    const coord_t min_y = aabb_.min_.Y - (aabb_.min_.Y % grid_size);

    for (coord_t x = min_x; x < aabb_.max_.X; x += grid_size)
    {
        writeLine(Point2LL(x, aabb_.min_.Y), Point2LL(x, aabb_.max_.Y), color, stroke_width);
        std::stringstream ss;
        ss << INT2MM(x);
        writeText(Point2LL(x, std::llrint(static_cast<double>(aabb_.min_.Y) + static_cast<double>(aabb_.max_.Y - aabb_.min_.Y) * dist_from_edge)), ss.str(), color, font_size);
    }
    for (coord_t y = min_y; y < aabb_.max_.Y; y += grid_size)
    {
        writeLine(Point2LL(aabb_.min_.X, y), Point2LL(aabb_.max_.Y, y), color, stroke_width);
        std::stringstream ss;
        ss << INT2MM(y);
        writeText(Point2LL(std::llrint(static_cast<double>(aabb_.min_.X) + static_cast<double>(aabb_.max_.X - aabb_.min_.X) * dist_from_edge), y), ss.str(), color, font_size);
    }
}

} // namespace cura
