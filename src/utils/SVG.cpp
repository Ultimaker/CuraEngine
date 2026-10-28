// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/SVG.h"

#include <sstream>

#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/c_str.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/transform.hpp>
#include <spdlog/spdlog.h>

#include "arachne/SkeletalTrapezoidationGraph.h"
#include "geometry/MixedLinesSet.h"
#include "geometry/OpenPolyline.h"
#include "geometry/Point2D.h"
#include "geometry/Polygon.h"
#include "geometry/SingleShape.h"
#include "geometry/conversions/Point2D_Point2LL.h"
#include "utils/ExtrusionLine.h"
#include "utils/VoronoiUtils.h"

namespace curaengine::utils
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


SVG::SVG(const std::string& filename, const AABB& aabb, const ::curaengine::geometry::Point2LL& canvas_size, const ColorObject& background)
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

SVG::SVG(const std::string& filename, const AABB& aabb, double scale, const ::curaengine::geometry::Point2LL& canvas_size, const ColorObject& background)
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

    if (! std::holds_alternative<std::monostate>(background))
    {
        std::string background_str = toString(background);
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

::curaengine::geometry::Point2LL SVG::transform(const ::curaengine::geometry::Point2LL& p) const
{
    return ::curaengine::geometry::Point2LL(std::llrint(static_cast<double>(p.X - aabb_.min_.X) * scale_), std::llrint(static_cast<double>(aabb_.max_.Y - p.Y) * scale_));
}

::curaengine::geometry::Point2D SVG::transformF(const ::curaengine::geometry::Point2LL& p) const
{
    return ::curaengine::geometry::Point2D(static_cast<double>(p.X - aabb_.min_.X) * scale_, static_cast<double>(aabb_.max_.Y - p.Y) * scale_);
}

void SVG::writeComment(const std::string& comment) const
{
    fprintf(out_, "<!-- %s -->\n", comment.c_str());
}

void SVG::write(const ::curaengine::geometry::Shape& shape, const VisualAttributes& visual_attributes, const bool flush) const
{
    write(static_cast<::curaengine::geometry::LinesSet<::curaengine::geometry::Polygon>>(shape), visual_attributes, flush, FillRule::EvenOdd);
}

template<class LineType>
void SVG::write(const ::curaengine::geometry::LinesSet<LineType>& lines, const VisualAttributes& visual_attributes, const bool flush, const FillRule fill_rule) const
{
    if (visual_attributes.line.isDisplayed() || visual_attributes.surface.isDisplayed())
    {
        if (visual_attributes.line.isRainbow())
        {
            // Rainbow can't be displayed with a path, we have to draw the segments separately
            write(lines, { .surface = visual_attributes.surface, .vertices = visual_attributes.vertices }, false);

            VisualAttributes rainbow_line_attributes{ .line = visual_attributes.line };

            for (const LineType& line : lines)
            {
                size_t index = 0;
                for (auto iterator = line.beginSegments(); iterator != line.endSegments(); ++iterator)
                {
                    rainbow_line_attributes.line.color = makeRainbowColor(index, line.segmentsCount());
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
                    fprintf(out_, "<path ");
                    write(visual_attributes.surface, fill_rule);
                    fprintf(
                        out_,
                        R"( stroke="%s" stroke-width="%f" stroke-dasharray="%s" stroke-opacity="%f" stroke-linecap="round" d=")",
                        toString(visual_attributes.line.color).c_str(),
                        visual_attributes.line.width,
                        toString(visual_attributes.line.dash_array).c_str(),
                        visual_attributes.line.alpha);
                    first_path = false;
                }
                writePathPoints(line);
            }

            fprintf(out_, "\" />\n"); // Write the end of the tag.
        }
    }

    for (const LineType& line : lines)
    {
        write(static_cast<::curaengine::geometry::PointsSet>(line), visual_attributes.vertices, false);
    }

    handleFlush(flush);
}

template void SVG::write(const ::curaengine::geometry::LinesSet<::curaengine::geometry::Polygon>& lines, const VisualAttributes& visual_attributes, const bool flush, const FillRule fill_rule) const;
template void SVG::write(const ::curaengine::geometry::LinesSet<::curaengine::geometry::OpenPolyline>& lines, const VisualAttributes& visual_attributes, const bool flush, const FillRule fill_rule) const;
template void SVG::write(const ::curaengine::geometry::LinesSet<::curaengine::geometry::ClosedPolyline>& lines, const VisualAttributes& visual_attributes, const bool flush, const FillRule fill_rule) const;

void SVG::write(const ::curaengine::geometry::OpenPolyline& line, const VisualAttributes& visual_attributes, const bool flush) const
{
    write(::curaengine::geometry::LinesSet{ line }, visual_attributes, flush);
}

void SVG::write(const ::curaengine::geometry::ClosedPolyline& line, const VisualAttributes& visual_attributes, const bool flush) const
{
    write(::curaengine::geometry::LinesSet{ line }, visual_attributes, flush);
}

void SVG::write(const ::curaengine::geometry::MixedLinesSet& lines, const VisualAttributes& visual_attributes, const bool flush) const
{
    for (const ::curaengine::geometry::PolylinePtr& line : lines)
    {
        if (auto open_polyline = dynamic_pointer_cast<::curaengine::geometry::OpenPolyline>(line))
        {
            write(*open_polyline, visual_attributes, false);
        }
        else if (auto closed_polyline = dynamic_pointer_cast<::curaengine::geometry::ClosedPolyline>(line))
        {
            write(*closed_polyline, visual_attributes, false);
        }
    }

    handleFlush(flush);
}

void SVG::write(const ::curaengine::geometry::PointsSet& points, const VerticesAttributes& visual_attributes, const bool flush) const
{
    for (const ::curaengine::geometry::Point2LL& point : points)
    {
        write(point, visual_attributes, false);
    }

    handleFlush(flush);
}

void SVG::write(const ::curaengine::geometry::Point2LL& point, const VerticesAttributes& visual_attributes, const bool flush) const
{
    if (visual_attributes.isDisplayed())
    {
        ::curaengine::geometry::Point2D transformed_point = transformF(point);
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

void SVG::write(const ::curaengine::geometry::Point2LL& start, const ::curaengine::geometry::Point2LL& end, const VisualAttributes& visual_attributes, const bool flush) const
{
    write(::curaengine::geometry::OpenPolyline({ start, end }), visual_attributes, flush);
}

void SVG::writeArrow(const ::curaengine::geometry::Point2LL& a, const ::curaengine::geometry::Point2LL& b, const ColorObject color, const double stroke_width, const double head_size) const
{
    const ::curaengine::geometry::Point2D fa = transformF(a);
    const ::curaengine::geometry::Point2D fb = transformF(b);
    const ::curaengine::geometry::Point2D ab = fb - fa;
    const ::curaengine::geometry::Point2D normal = ::curaengine::geometry::Point2D(ab.y(), -ab.x()).vNormalized().value();
    const ::curaengine::geometry::Point2D direction = ab.vNormalized().value();

    const ::curaengine::geometry::Point2D tip = fb + normal * head_size - direction * head_size;
    const ::curaengine::geometry::Point2D b_base = fb + normal * stroke_width - direction * stroke_width * 2.41f;
    const ::curaengine::geometry::Point2D a_base = fa + normal * stroke_width;
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

void SVG::write(const std::string& text, const ::curaengine::geometry::Point2LL& p, const VerticesAttributes& vertices_attributes, const bool flush) const
{
    const ::curaengine::geometry::Point2D transformed_point = transformF(p);
    fprintf(
        out_,
        "<text x=\"%f\" y=\"%f\" style=\"font-size: %fpx;\" fill=\"%s\">%s</text>\n",
        transformed_point.x(),
        transformed_point.y(),
        vertices_attributes.font_size * scale_,
        toString(vertices_attributes.color).c_str(),
        text.c_str());

    handleFlush(flush);
}

void SVG::write(const ::curaengine::arachne::SkeletalTrapezoidationGraph& graph, const STVisualAttributes& visual_attributes, const bool flush) const
{
    const size_t start_edges_count = ranges::accumulate(
        graph.edges_,
        0,
        [](int total, const ::curaengine::arachne::STHalfEdge& edge)
        {
            return total + (edge.prev_ == nullptr ? 1 : 0);
        });

    std::set<const ::curaengine::arachne::STHalfEdge*> drawn_edges;
    std::set<const ::curaengine::arachne::STHalfEdgeNode*> drawn_nodes;

    for (const auto& [index, edge] : graph.edges_ | ranges::views::enumerate)
    {
        if (edge.prev_ != nullptr)
        {
            continue;
        }

        STVisualAttributes loop_visual_attributes = visual_attributes;
        if (loop_visual_attributes.line.isRainbow())
        {
            loop_visual_attributes.line.color = makeRainbowColor(index, start_edges_count);
        }

        const ::curaengine::arachne::STHalfEdge* current_edge = &edge;
        while (current_edge != nullptr)
        {
            if (! drawn_edges.contains(current_edge))
            {
                write(*current_edge, loop_visual_attributes, false, drawn_nodes);

                STVisualAttributes twin_visual_attributes = loop_visual_attributes;
                twin_visual_attributes.line = LineAttributes::hidden();
                if (current_edge->twin_)
                {
                    write(*current_edge->twin_, twin_visual_attributes, false, drawn_nodes);
                }

                if (! visual_attributes.edges_arrows)
                {
                    drawn_edges.emplace(current_edge);
                    drawn_edges.emplace(current_edge->twin_);
                }

                if (visual_attributes.beads_count.isDisplayed())
                {
                    drawn_nodes.emplace(current_edge->from_);
                    drawn_nodes.emplace(current_edge->to_);
                }
            }

            current_edge = current_edge->next_;
        }
    }

    handleFlush(flush);
}

void SVG::write(const ::curaengine::arachne::STHalfEdge& edge, const STVisualAttributes& visual_attributes, const bool flush, const std::set<const ::curaengine::arachne::STHalfEdgeNode*>& drawn_nodes) const
{
    if (visual_attributes.beads_count.isDisplayed())
    {
        if (! drawn_nodes.contains(edge.from_))
        {
            write(std::to_string(edge.from_->data_.bead_count_), edge.from_->p_, visual_attributes.beads_count, false);
        }
        if (! drawn_nodes.contains(edge.to_))
        {
            write(std::to_string(edge.to_->data_.bead_count_), edge.to_->p_, visual_attributes.beads_count, false);
        }
    }

    if (visual_attributes.junctions.isDisplayed() && edge.data_.hasExtrusionJunctions())
    {
        std::shared_ptr<LineJunctions> junctions = edge.data_.getExtrusionJunctions();
        for (const ExtrusionJunction& junction : *junctions)
        {
            write(junction.p_, visual_attributes.junctions);
        }
    }

    if (! visual_attributes.edges_arrows)
    {
        write(edge.from_->p_, edge.to_->p_, visual_attributes, flush);
        return;
    }

    const std::optional<::curaengine::geometry::Point2D> direction = ::curaengine::geometry::conversions::toPoint2D(::curaengine::geometry::Point2LL(edge.to_->p_ - edge.from_->p_)).vNormalized();
    if (! direction.has_value())
    {
        spdlog::warn("Invalid half edge, skipping");
        return;
    }

    const ::curaengine::geometry::Point2D radial = direction->rotated90CCW();
    const ::curaengine::geometry::Point2LL delta_radial = ::curaengine::geometry::conversions::toPoint2LL(radial * (visual_attributes.line.width * 0.65 / scale_));
    const ::curaengine::geometry::Point2LL size_reduction_axial = ::curaengine::geometry::conversions::toPoint2LL(direction.value() * (visual_attributes.line.width * 4.0 / scale_));

    ::curaengine::geometry::OpenPolyline edge_half_arrow;
    edge_half_arrow.push_back(edge.from_->p_ + size_reduction_axial + delta_radial);
    edge_half_arrow.push_back(edge.to_->p_ - size_reduction_axial + delta_radial);

    const ::curaengine::geometry::Point2LL tip_delta = ::curaengine::geometry::conversions::toPoint2LL(-(direction.value() - radial) * (visual_attributes.line.width * 1.5 / scale_));
    edge_half_arrow.push_back(edge_half_arrow.back() + tip_delta);

    write(edge_half_arrow, visual_attributes, flush);
}

void SVG::write(const std::vector<VariableWidthLines>& paths, const SurfaceAttributes& visual_attributes, const bool flush) const
{
    for (const VariableWidthLines& lines : paths)
    {
        write(lines, visual_attributes, false);
    }

    handleFlush(flush);
}

void SVG::write(const VariableWidthLines& lines, const SurfaceAttributes& visual_attributes, const bool flush) const
{
    for (const ExtrusionLine& line : lines)
    {
        write(line, visual_attributes, false);
    }

    handleFlush(flush);
}

void SVG::write(const ExtrusionLine& line, const SurfaceAttributes& visual_attributes, const bool flush) const
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
        const ::curaengine::geometry::Point2LL direction_vector = end_vertex.p_ - start_vertex.p_;
        const ::curaengine::geometry::Point2LL direction_left = ::curaengine::geometry::turn90CCW(direction_vector);
        const ::curaengine::geometry::Point2LL direction_right = -direction_left; // Opposite of left.
        const ::curaengine::geometry::Point2D start_left = transformF(start_vertex.p_ + ::curaengine::geometry::normal(direction_left, std::llrint(std::max(minimum_line_width, static_cast<double>(start_vertex.w_) / 2.0))));
        const ::curaengine::geometry::Point2D start_right = transformF(start_vertex.p_ + ::curaengine::geometry::normal(direction_right, std::llrint(std::max(minimum_line_width, static_cast<double>(start_vertex.w_) / 2.0))));
        const ::curaengine::geometry::Point2D end_left = transformF(end_vertex.p_ + ::curaengine::geometry::normal(direction_left, std::llrint(std::max(minimum_line_width, static_cast<double>(end_vertex.w_) / 2.0))));
        const ::curaengine::geometry::Point2D end_right = transformF(end_vertex.p_ + ::curaengine::geometry::normal(direction_right, std::llrint(std::max(minimum_line_width, static_cast<double>(end_vertex.w_) / 2.0))));

        fprintf(out_, "<polygon ");
        write(visual_attributes);
        fprintf(
            out_,
            " points=\"%f,%f %f,%f %f,%f %f,%f\" />\n",
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

    handleFlush(flush);
}

void SVG::write(const SurfaceAttributes& visual_attributes, const FillRule fill_rule) const
{
    fprintf(out_, R"(fill="%s" fill-opacity="%f" fill-rule="%s")", toString(visual_attributes.color).c_str(), visual_attributes.alpha, toString(fill_rule).c_str());
}

void SVG::write(const LineAttributes& visual_attributes) const
{
    fprintf(
        out_,
        R"(stroke="%s" stroke-width="%f" stroke-dasharray="%s" stroke-opacity="%f" stroke-linecap="round")",
        toString(visual_attributes.color).c_str(),
        visual_attributes.width,
        toString(visual_attributes.dash_array).c_str(),
        visual_attributes.alpha);
}

void SVG::writeCoordinateGrid(const coord_t grid_size, const VisualAttributes& visual_attributes, const bool flush) const
{
    constexpr double dist_from_edge = 0.05; // As fraction of image width or height.
    const coord_t min_x = aabb_.min_.X - (aabb_.min_.X % grid_size);
    const coord_t min_y = aabb_.min_.Y - (aabb_.min_.Y % grid_size);

    for (coord_t x = min_x; x < aabb_.max_.X; x += grid_size)
    {
        write(::curaengine::geometry::Point2LL(x, aabb_.min_.Y), ::curaengine::geometry::Point2LL(x, aabb_.max_.Y), visual_attributes);
        std::stringstream ss;
        ss << INT2MM(x);
        write(
            ss.str(),
            ::curaengine::geometry::Point2LL(x, std::llrint(static_cast<double>(aabb_.min_.Y) + static_cast<double>(aabb_.max_.Y - aabb_.min_.Y) * dist_from_edge)),
            visual_attributes.vertices,
            false);
    }
    for (coord_t y = min_y; y < aabb_.max_.Y; y += grid_size)
    {
        write(::curaengine::geometry::Point2LL(aabb_.min_.X, y), ::curaengine::geometry::Point2LL(aabb_.max_.Y, y), visual_attributes);
        std::stringstream ss;
        ss << INT2MM(y);
        write(
            ss.str(),
            ::curaengine::geometry::Point2LL(std::llrint(static_cast<double>(aabb_.min_.X) + static_cast<double>(aabb_.max_.X - aabb_.min_.X) * dist_from_edge), y),
            visual_attributes.vertices,
            false);
    }

    handleFlush(flush);
}

template<>
void SVG::write(const boost::polygon::voronoi_edge<VoronoiUtils::voronoi_data_t>& edge, const DiagramVisualAttributes& visual_attributes, const bool flush) const
{
    if (edge.is_infinite())
    {
        return;
    }

    const auto& v0 = edge.vertex0();
    const auto& v1 = edge.vertex1();

    if (v0 == nullptr || v1 == nullptr)
    {
        return;
    }

    const ::curaengine::geometry::Point2LL p0 = VoronoiUtils::p(v0);
    const ::curaengine::geometry::Point2LL p1 = VoronoiUtils::p(v1);

    ::curaengine::arachne::STHalfEdge half_edge = { ::curaengine::arachne::SkeletalTrapezoidationEdge() };
    ::curaengine::arachne::STHalfEdgeNode from(::curaengine::arachne::SkeletalTrapezoidationJoint(), p0);
    ::curaengine::arachne::STHalfEdgeNode to(::curaengine::arachne::SkeletalTrapezoidationJoint(), p1);
    half_edge.from_ = &from;
    half_edge.to_ = &to;

    write(half_edge, visual_attributes, flush);
}

template<>
void SVG::write(const boost::polygon::voronoi_cell<VoronoiUtils::voronoi_data_t>& cell, const DiagramVisualAttributes& visual_attributes, const bool flush) const
{
    const VoronoiUtils::vd_t::edge_type* start_edge = cell.incident_edge();
    if (! start_edge)
    {
        return;
    }

    ::curaengine::geometry::Polygon cell_polygon;

    const VoronoiUtils::vd_t::edge_type* current_edge = start_edge;
    do
    {
        if (current_edge->is_finite())
        {
            if (visual_attributes.line.isDisplayed() || visual_attributes.vertices.isDisplayed())
            {
                write(*current_edge, visual_attributes, false);
            }

            if (visual_attributes.surface.isDisplayed())
            {
                const auto& v1 = current_edge->vertex1();
                if (v1 != nullptr)
                {
                    cell_polygon.push_back(VoronoiUtils::p(v1));
                }
            }
        }
        current_edge = current_edge->next();
    } while (current_edge && current_edge != start_edge);

    if (cell_polygon.isValid())
    {
        VisualAttributes cell_surface_attributes;
        cell_surface_attributes.surface = visual_attributes.surface;
        write(cell_polygon, cell_surface_attributes, false);
    }

    handleFlush(flush);
}

template<>
void SVG::write(const VoronoiUtils::vd_t& voronoi_diagram, const DiagramVisualAttributes& visual_attributes, const bool flush) const
{
    for (const auto& [index, cell] : voronoi_diagram.cells() | ranges::views::enumerate)
    {
        DiagramVisualAttributes cell_visual_attributes = visual_attributes;
        if (cell_visual_attributes.line.isRainbow())
        {
            cell_visual_attributes.line.color = makeRainbowColor(index, voronoi_diagram.cells().size());
        }

        write(cell, cell_visual_attributes, false);
    }

    handleFlush(flush);
}

void SVG::writePathPoints(const ::curaengine::geometry::Polyline& line) const
{
    bool first_segment = true;
    const bool has_closing_segment = line.hasClosingSegment();
    const auto before_last_iterator = std::next(line.endSegments(), -1);
    for (auto iterator = line.beginSegments(); iterator != line.endSegments(); ++iterator)
    {
        if (first_segment)
        {
            const ::curaengine::geometry::Point2D transformed_start = transformF((*iterator).start);
            fprintf(out_, "M%f %f ", transformed_start.x(), transformed_start.y());
            first_segment = false;
        }

        if (! has_closing_segment || iterator != before_last_iterator)
        {
            const ::curaengine::geometry::Point2D transformed_end = transformF((*iterator).end);
            fprintf(out_, "L%f,%f ", static_cast<double>(transformed_end.x()), static_cast<double>(transformed_end.y()));
        }
    }

    if (has_closing_segment)
    {
        fprintf(out_, "Z ");
    }
}

SVG::RgbColor SVG::makeRainbowColor(const size_t index, const size_t elementsCount)
{
    RgbColor color;
    color.r = index * 255 / elementsCount;
    color.g = (index * 255 * 11 / elementsCount) % (255 * 2);
    if (color.g > 255)
    {
        color.g = 255 * 2 - color.g;
    }
    color.b = (index * 255 * 5 / elementsCount) % (255 * 2);
    if (color.b > 255)
    {
        color.b = 255 * 2 - color.b;
    }

    return color;
}

} // namespace curaengine::utils
