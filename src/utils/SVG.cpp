// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/SVG.h"

#include <sstream>

#include <range/v3/to_container.hpp>
#include <range/v3/view/c_str.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/transform.hpp>
#include <spdlog/spdlog.h>

#include "geometry/OpenPolyline.h"
#include "geometry/Point2D.h"
#include "geometry/Polygon.h"
#include "geometry/SingleShape.h"
#include "utils/ExtrusionLine.h"

namespace cura
{


std::string SVG::toString(const Color color)
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
    case SVG::Color::RAINBOW: // rainbow case should never be reached
        return "none";
    default:
        return "black";
    }
}

SVG::RgbColor SVG::toRgb(const Color color)
{
    switch (color)
    {
    case SVG::Color::WHITE:
        return RgbColor(255, 255, 255);
    case SVG::Color::GRAY:
        return RgbColor(128, 128, 128);
    case SVG::Color::RED:
        return RgbColor(255, 0, 0);
    case SVG::Color::BLUE:
        return RgbColor(0, 0, 255);
    case SVG::Color::GREEN:
        return RgbColor(0, 255, 0);
    case SVG::Color::LIME:
        return RgbColor(191, 255, 0);
    case SVG::Color::ORANGE:
        return RgbColor(255, 165, 0);
    case SVG::Color::MAGENTA:
        return RgbColor(255, 0, 255);
    case SVG::Color::YELLOW:
        return RgbColor(255, 255, 0);
    case SVG::Color::BLACK:
    case SVG::Color::RAINBOW: // rainbow case should never be reached
    case SVG::Color::NONE:
        return RgbColor(0, 0, 0);
    }

    return RgbColor{};
}

std::string SVG::toString(const ColorObject& color)
{
    if (std::holds_alternative<Color>(color))
    {
        return toString(std::get<Color>(color));
    }

    if (std::holds_alternative<RgbColor>(color))
    {
        const RgbColor& rgb_color = std::get<RgbColor>(color);
        std::ostringstream ss;
        ss << "rgb(" << rgb_color.r << "," << rgb_color.g << "," << rgb_color.b << ")";
        return ss.str();
    }

    return "none";
}

std::string SVG::toString(const std::vector<int>& dash_array)
{
    if (dash_array.size() >= 2)
    {
        return dash_array
             | ranges::views::transform(
                   [](const int dash_array_part)
                   {
                       return std::to_string(dash_array_part);
                   })
             | ranges::views::join(ranges::views::c_str(",")) | ranges::to<std::string>();
    }

    return "none";
}

std::string SVG::toString(const FillRule fill_rule)
{
    switch (fill_rule)
    {
    case FillRule::NonZero:
        return "nonzero";
    case FillRule::EvenOdd:
        return "evenodd";
    case FillRule::None:
        break;
    }

    return "none";
}

void SVG::handleFlush(const bool flush) const
{
    if (flush)
    {
        fflush(out_);
    }
}


SVG::SVG(const std::string& filename, const AABB& aabb, const Point2LL& canvas_size, const ColorObject& background)
    : SVG(filename,
          aabb,
          std::min(
              static_cast<double>(canvas_size.X - canvas_size.X / 5 * 2) / static_cast<double>(aabb.max_.X - aabb.min_.X),
              static_cast<double>(canvas_size.Y - canvas_size.Y / 5) / static_cast<double>(aabb.max_.Y - aabb.min_.Y)),
          canvas_size,
          background)
{
}

SVG::SVG(const std::string& filename, const AABB& aabb, double scale, const ColorObject& background)
    : SVG(filename, aabb, scale, (aabb.max_ - aabb.min_) * scale, background)
{
}

SVG::SVG(const std::string& filename, const AABB& aabb, double scale, const Point2LL& canvas_size, const ColorObject& background)
    : aabb_(aabb)
    , aabb_size_(aabb.max_ - aabb.min_)
    , canvas_size_(canvas_size)
    , scale_(scale)
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

    std::string background_str = toString(background);
    if (! background_str.empty())
    {
        fprintf(out_, "<rect width=\"100%%\" height=\"100%%\" fill=\"%s\"/>\n", background_str.c_str());
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
    return Point2LL(std::llrint(static_cast<double>(p.X - aabb_.min_.X) * scale_), std::llrint(static_cast<double>(aabb_.max_.Y - p.Y) * scale_));
}

Point2D SVG::transformF(const Point2LL& p) const
{
    return Point2D(static_cast<double>(p.X - aabb_.min_.X) * scale_, static_cast<double>(aabb_.max_.Y - p.Y) * scale_);
}

void SVG::writeComment(const std::string& comment) const
{
    fprintf(out_, "<!-- %s -->\n", comment.c_str());
}

void SVG::write(const Shape& shape, const VisualAttributes& visual_attributes, const bool flush) const
{
    write(static_cast<LinesSet<Polygon>>(shape), visual_attributes, flush, FillRule::EvenOdd);
}

template<class LineType>
void SVG::write(const LinesSet<LineType>& lines, const VisualAttributes& visual_attributes, const bool flush, const FillRule fill_rule) const
{
    if (visual_attributes.line.isDisplayed() || visual_attributes.surface.isDisplayed())
    {
        if (std::holds_alternative<Color>(visual_attributes.line.color) && std::get<Color>(visual_attributes.line.color) == Color::RAINBOW)
        {
            // Rainbow can't be displayed with a path, we have to draw the segments separately
            write(lines, { .surface = visual_attributes.surface, .vertices = visual_attributes.vertices }, false);

            VisualAttributes rainbow_line_attributes{ .line = visual_attributes.line };

            for (const LineType& line : lines)
            {
                size_t index = 0;
                for (auto iterator = line.beginSegments(); iterator != line.endSegments(); ++iterator)
                {
                    RgbColor color;
                    color.r = index * 255 / line.segmentsCount();
                    color.g = (index * 255 * 11 / line.segmentsCount()) % (255 * 2);
                    if (color.g > 255)
                    {
                        color.g = 255 * 2 - color.g;
                    }
                    color.b = (index * 255 * 5 / line.segmentsCount()) % (255 * 2);
                    if (color.b > 255)
                    {
                        color.b = 255 * 2 - color.b;
                    }

                    rainbow_line_attributes.line.color = color;
                    write((*iterator).start, (*iterator).end, rainbow_line_attributes, false);
                    index++;
                }
            }
        }
        else
        {
            bool first_path = true;
            for (const LineType& line : lines)
            {
                if (first_path)
                {
                    fprintf(
                        out_,
                        "<path fill=\"%s\" fill-rule=\"%s\" stroke=\"%s\" stroke-width=\"%f\" stroke-dasharray=\"%s\" d=\"",
                        toString(visual_attributes.surface.color).c_str(),
                        toString(fill_rule).c_str(),
                        toString(visual_attributes.line.color).c_str(),
                        visual_attributes.line.width,
                        toString(visual_attributes.line.dash_array).c_str());
                    first_path = false;
                }
                writePathPoints(line);
            }

            fprintf(out_, "\" />\n"); // Write the end of the tag.
        }

        handleFlush(flush);
    }

    for (const LineType& line : lines)
    {
        write(static_cast<PointsSet>(line), visual_attributes.vertices, false);
    }

    handleFlush(flush);
}

template void SVG::write(const LinesSet<Polygon>& lines, const VisualAttributes& visual_attributes, const bool flush, const FillRule fill_rule) const;
template void SVG::write(const LinesSet<OpenPolyline>& lines, const VisualAttributes& visual_attributes, const bool flush, const FillRule fill_rule) const;
template void SVG::write(const LinesSet<ClosedPolyline>& lines, const VisualAttributes& visual_attributes, const bool flush, const FillRule fill_rule) const;

void SVG::write(const OpenPolyline& line, const VisualAttributes& visual_attributes, const bool flush) const
{
    write(LinesSet{ line }, visual_attributes, flush);
}

void SVG::write(const ClosedPolyline& line, const VisualAttributes& visual_attributes, const bool flush) const
{
    write(LinesSet{ line }, visual_attributes, flush);
}

void SVG::write(const PointsSet& points, const VerticesAttributes& visual_attributes, const bool flush) const
{
    for (const Point2LL& point : points)
    {
        write(point, visual_attributes, false);
    }

    handleFlush(flush);
}

void SVG::write(const Point2LL& point, const VerticesAttributes& visual_attributes, const bool flush) const
{
    if (visual_attributes.isDisplayed())
    {
        Point2D transformed_point = transformF(point);
        fprintf(
            out_,
            "<circle cx=\"%f\" cy=\"%f\" r=\"%f\" stroke-width=\"0\" fill=\"%s\" />\n",
            static_cast<double>(transformed_point.x()),
            static_cast<double>(transformed_point.y()),
            static_cast<double>(visual_attributes.radius),
            toString(visual_attributes.color).c_str());

        if (visual_attributes.write_coords)
        {
            write(fmt::format("{},{}", point.X, point.Y), point, visual_attributes, false);
        }
    }

    handleFlush(flush);
}

void SVG::write(const Point2LL& start, const Point2LL& end, const VisualAttributes& visual_attributes, const bool flush) const
{
    write(OpenPolyline({ start, end }), visual_attributes, flush);
}

void SVG::writeArrow(const Point2LL& a, const Point2LL& b, const ColorObject color, const double stroke_width, const double head_size) const
{
    const Point2D fa = transformF(a);
    const Point2D fb = transformF(b);
    const Point2D ab = fb - fa;
    const Point2D normal = Point2D(ab.y(), -ab.x()).vNormalized().value();
    const Point2D direction = ab.vNormalized().value();

    const Point2D tip = fb + normal * head_size - direction * head_size;
    const Point2D b_base = fb + normal * stroke_width - direction * stroke_width * 2.41f;
    const Point2D a_base = fa + normal * stroke_width;
    fprintf(
        out_,
        "<polygon fill=\"%s\" points=\"%f,%f %f,%f %f,%f %f,%f %f,%f\" />",
        toString(color).c_str(),
        static_cast<double>(fa.x()),
        static_cast<double>(fa.y()),
        static_cast<double>(fb.x()),
        static_cast<double>(fb.y()),
        static_cast<double>(tip.x()),
        static_cast<double>(tip.y()),
        static_cast<double>(b_base.x()),
        static_cast<double>(b_base.y()),
        static_cast<double>(a_base.x()),
        static_cast<double>(a_base.y()));
}

void SVG::write(const std::string& text, const Point2LL& p, const VerticesAttributes& vertices_attributes, const bool flush) const
{
    const Point2D transformed_point = transformF(p);
    fprintf(
        out_,
        "<text x=\"%f\" y=\"%f\" style=\"font-size: %fpx;\" fill=\"%s\">%s</text>\n",
        transformed_point.x(),
        transformed_point.y(),
        vertices_attributes.font_size,
        toString(vertices_attributes.color).c_str(),
        text.c_str());

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
        const Point2D start_left
            = transformF(start_vertex.p_ + normal(direction_left, std::llrint(std::max(minimum_line_width, static_cast<double>(start_vertex.w_) * width_factor))));
        const Point2D start_right
            = transformF(start_vertex.p_ + normal(direction_right, std::llrint(std::max(minimum_line_width, static_cast<double>(start_vertex.w_) * width_factor))));
        const Point2D end_left = transformF(end_vertex.p_ + normal(direction_left, std::llrint(std::max(minimum_line_width, static_cast<double>(end_vertex.w_) * width_factor))));
        const Point2D end_right = transformF(end_vertex.p_ + normal(direction_right, std::llrint(std::max(minimum_line_width, static_cast<double>(end_vertex.w_) * width_factor))));

        fprintf(
            out_,
            "<polygon fill=\"%s\" points=\"%f,%f %f,%f %f,%f %f,%f\" />\n",
            toString(color).c_str(),
            static_cast<double>(start_left.x()),
            static_cast<double>(start_left.y()),
            static_cast<double>(start_right.x()),
            static_cast<double>(start_right.y()),
            static_cast<double>(end_right.x()),
            static_cast<double>(end_right.y()),
            static_cast<double>(end_left.x()),
            static_cast<double>(end_left.y()));

        start_vertex = end_vertex; // For the next line segment.
    }

    if (flush)
    {
        fflush(out_);
    }
}

void SVG::writeCoordinateGrid(const coord_t grid_size, const VisualAttributes& visual_attributes, const bool flush) const
{
    constexpr double dist_from_edge = 0.05; // As fraction of image width or height.
    const coord_t min_x = aabb_.min_.X - (aabb_.min_.X % grid_size);
    const coord_t min_y = aabb_.min_.Y - (aabb_.min_.Y % grid_size);

    for (coord_t x = min_x; x < aabb_.max_.X; x += grid_size)
    {
        write(Point2LL(x, aabb_.min_.Y), Point2LL(x, aabb_.max_.Y), visual_attributes);
        std::stringstream ss;
        ss << INT2MM(x);
        write(
            ss.str(),
            Point2LL(x, std::llrint(static_cast<double>(aabb_.min_.Y) + static_cast<double>(aabb_.max_.Y - aabb_.min_.Y) * dist_from_edge)),
            visual_attributes.vertices,
            false);
    }
    for (coord_t y = min_y; y < aabb_.max_.Y; y += grid_size)
    {
        write(Point2LL(aabb_.min_.X, y), Point2LL(aabb_.max_.Y, y), visual_attributes);
        std::stringstream ss;
        ss << INT2MM(y);
        write(
            ss.str(),
            Point2LL(std::llrint(static_cast<double>(aabb_.min_.X) + static_cast<double>(aabb_.max_.X - aabb_.min_.X) * dist_from_edge), y),
            visual_attributes.vertices,
            false);
    }

    handleFlush(flush);
}

void SVG::writePathPoints(const Polyline& line) const
{
    bool first_segment = true;
    for (auto iterator = line.beginSegments(); iterator != line.endSegments(); ++iterator)
    {
        if (first_segment)
        {
            const Point2D transformed_start = transformF((*iterator).start);
            fprintf(out_, "M%f %f ", transformed_start.x(), transformed_start.y());
            first_segment = false;
        }

        const Point2D transformed_end = transformF((*iterator).end);
        fprintf(out_, "L%f,%f ", static_cast<double>(transformed_end.x()), static_cast<double>(transformed_end.y()));
    }
}

} // namespace cura
