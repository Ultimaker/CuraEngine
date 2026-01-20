// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SVG_H
#define SVG_H

#include <stdio.h> // for file output
#include <variant>

#include <boost/polygon/voronoi.hpp>

#include "AABB.h"
#include "ExtrusionLine.h" //To accept variable-width paths.
#include "NoCopy.h"
#include "geometry/Point2LL.h"

namespace cura
{

class Point2D;
class MixedLinesSet;
class SkeletalTrapezoidationGraph;
class STHalfEdge;

class SVG : NoCopy
{
public:
    enum class Color
    {
        BLACK,
        WHITE,
        GRAY,
        RED,
        BLUE,
        GREEN,
        LIME,
        ORANGE,
        MAGENTA,
        YELLOW,
        RAINBOW, // used for making the color change throughout the polygon which is drawn
        NONE
    };

    enum class FillRule
    {
        None,
        NonZero,
        EvenOdd
    };

    struct RgbColor
    {
        int r{ 0 };
        int g{ 0 };
        int b{ 0 };

        RgbColor() = default;
        RgbColor(int red, int green, int blue)
            : r(red)
            , g(green)
            , b(blue)
        {
        }
    };

    using ColorObject = std::variant<std::monostate, Color, RgbColor>;

    struct ElementAttributes
    {
        ColorObject color{ Color::BLACK };

        ElementAttributes() = default;

        ElementAttributes(const ColorObject& color)
            : color(color)
        {
        }

        virtual ~ElementAttributes() = default;

        virtual bool isDisplayed() const
        {
            return (std::holds_alternative<Color>(color) && std::get<Color>(color) != Color::NONE) || std::holds_alternative<RgbColor>(color);
        }

        bool isRainbow() const
        {
            return std::holds_alternative<Color>(color) && std::get<Color>(color) == Color::RAINBOW;
        }
    };

    struct SurfaceAttributes : ElementAttributes
    {
        double alpha{ 1.0 };

        SurfaceAttributes() = default;

        SurfaceAttributes(const ColorObject& color)
            : ElementAttributes(color)
        {
        }

        SurfaceAttributes(const ColorObject& color, const double alpha)
            : ElementAttributes(color)
            , alpha(alpha)
        {
        }

        ~SurfaceAttributes() override = default;
    };

    struct LineAttributes : ElementAttributes
    {
        double width{ 0.4 };
        std::vector<int> dash_array{};

        LineAttributes() = default;

        LineAttributes(const ColorObject& color, const double width)
            : ElementAttributes(color)
            , width(width)
        {
        }

        LineAttributes(const ColorObject& color)
            : ElementAttributes(color)
        {
        }

        LineAttributes(const double width)
            : ElementAttributes()
            , width(width)
        {
        }

        ~LineAttributes() override = default;

        bool isDisplayed() const override
        {
            return ElementAttributes::isDisplayed() && width > 0.0;
        }
    };

    struct VerticesAttributes : ElementAttributes
    {
        double radius{ 0.2 };
        bool write_coords{ false };
        double font_size{ 10 };

        VerticesAttributes() = default;

        VerticesAttributes(const ColorObject& color, const double radius)
            : ElementAttributes(color)
            , radius(radius)
        {
        }

        VerticesAttributes(const ColorObject& color)
            : ElementAttributes(color)
        {
        }

        VerticesAttributes(const double radius)
            : ElementAttributes()
            , radius(radius)
        {
        }

        VerticesAttributes(const bool write_coords)
            : ElementAttributes()
            , write_coords(write_coords)
        {
        }

        ~VerticesAttributes() override = default;

        bool isDisplayed() const override
        {
            return ElementAttributes::isDisplayed() && (radius > 0.0 || (write_coords && font_size > 0.0));
        }
    };

    struct VisualAttributes
    {
        SurfaceAttributes surface{ Color::NONE };
        LineAttributes line{ Color::NONE, 0.0 };
        VerticesAttributes vertices{ Color::NONE, 0.0 };
    };

private:
    static std::string toString(const ColorObject& color);
    static std::string toString(const std::vector<int>& dash_array);
    static std::string toString(const FillRule fill_rule);
    void handleFlush(const bool flush) const;

    FILE* out_; // the output file
    const AABB aabb_; // the boundary box to display
    const Point2LL aabb_size_;
    const Point2LL canvas_size_;
    const double scale_;
    size_t layer_nr_ = 1;

    bool output_is_html_;

public:
    SVG(const std::string& filename, const AABB& aabb, const Point2LL& canvas_size = Point2LL(1024, 1024), const ColorObject& background = Color::NONE);
    SVG(const std::string& filename, const AABB& aabb, const double scale, const ColorObject& background = Color::NONE);
    SVG(const std::string& filename, const AABB& aabb, const double scale, const Point2LL& canvas_size, const ColorObject& background = Color::NONE);

    ~SVG();

    static std::string toString(const Color color);

    static RgbColor toRgb(const Color color);

    /*!
     * get the scaling factor applied to convert real space to canvas space
     */
    double getScale() const;

    void nextLayer();

    /*!
     * transform a point in real space to canvas space
     */
    Point2LL transform(const Point2LL& p) const;

    /*!
     * transform a point in real space to canvas space with more precision
     */
    Point2D transformF(const Point2LL& p) const;

    void writeComment(const std::string& comment) const;

    void write(const Shape& shape, const VisualAttributes& visual_attributes, const bool flush = true) const;

    template<class LineType>
    void write(const LinesSet<LineType>& lines, const VisualAttributes& visual_attributes, const bool flush = true, const FillRule fill_rule = FillRule::None) const;

    void write(const OpenPolyline& line, const VisualAttributes& visual_attributes, const bool flush = true) const;

    void write(const ClosedPolyline& line, const VisualAttributes& visual_attributes, const bool flush = true) const;

    void write(const MixedLinesSet& lines, const VisualAttributes& visual_attributes, const bool flush = true) const;

    void write(const Point2LL& start, const Point2LL& end, const VisualAttributes& visual_attributes, const bool flush = true) const;

    void write(const PointsSet& points, const VerticesAttributes& visual_attributes, const bool flush = true) const;

    void write(const Point2LL& point, const VerticesAttributes& visual_attributes, const bool flush = true) const;

    void write(const std::string& text, const Point2LL& p, const VerticesAttributes& vertices_attributes, const bool flush = true) const;

    void write(const SkeletalTrapezoidationGraph& graph, const VisualAttributes& visual_attributes, const bool flush = true) const;

    void write(const STHalfEdge& edge, const VisualAttributes& visual_attributes, const bool flush = true) const;

    template<typename T>
    void write(const boost::polygon::voronoi_diagram<T>& voronoi_diagram, const VisualAttributes& visual_attributes, const bool flush = true) const;

    template<typename T>
    void write(const boost::polygon::voronoi_edge<T>& edge, const VisualAttributes& visual_attributes, const bool flush = true) const;

    template<typename T>
    void write(const boost::polygon::voronoi_cell<T>& cell, const VisualAttributes& visual_attributes, const bool flush = true) const;

    void writeArrow(const Point2LL& a, const Point2LL& b, const ColorObject color = Color::BLACK, const double stroke_width = 1.0, const double head_size = 5.0) const;

    template<typename... Args>
    void printf(const char* txt, Args&&... args) const;

    /*!
     * Draw variable-width paths into the image.
     *
     * The paths are drawn with the correct line width, as given in the paths,
     * but there is a multiplicative factor to adjust the width with.
     * \param paths The paths to draw.
     * \param color The color to draw the paths with.
     * \param width_factor A multiplicative factor on the line widths.
     */
    void writePaths(const std::vector<VariableWidthLines>& paths, const ColorObject color = Color::BLACK, const double width_factor = 1.0) const;

    /*!
     * Draw variable-width lines into the image.
     *
     * The lines are drawn with the correct line width, as given in the lines,
     * but there is a multiplicative factor to adjust the width with.
     * \param lines The lines to draw.
     * \param color The color to draw the lines with.
     * \param width_factor A multiplicative factor on the line widths.
     */
    void writeLines(const VariableWidthLines& lines, const ColorObject color = Color::BLACK, const double width_factor = 1.0) const;

    /*!
     * Draw a variable-width line into the image.
     *
     * The line is drawn with the correct line width, as given in the junctions,
     * but there is a multiplicative factor to adjust the width with.
     * \param line The line to draw.
     * \param color The color to draw the line with.
     * \param width_factor A multiplicative factor on the line width.
     */
    void writeLine(const ExtrusionLine& line, const ColorObject color = Color::BLACK, const double width_factor = 1.0, const bool flush = true) const;

    /*!
     * Draws a grid across the image and writes down coordinates.
     *
     * Coordinates are always written in millimeters.
     * \param grid_size Size of the grid cells.
     * \param color The colour to draw the grid with.
     * \param stroke_width The width of the grid lines.
     * \param font_size The size of the font to write the coordinates with.
     */
    void writeCoordinateGrid(const coord_t grid_size, const VisualAttributes& visual_attributes, const bool flush = true) const;

private:
    void writePathPoints(const Polyline& line) const;

    static RgbColor makeRainbowColor(const size_t index, const size_t elementsCount);
};

template<typename... Args>
void SVG::printf(const char* txt, Args&&... args) const
{
    fprintf(out_, txt, args...);
}

} // namespace cura
#endif // SVG_H
