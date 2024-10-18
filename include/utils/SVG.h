// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SVG_H
#define SVG_H

#include <concepts>
#include <stdio.h> // for file output

#include <boost/polygon/voronoi.hpp>

#include "AABB.h"
#include "ExtrusionLine.h" //To accept variable-width paths.
#include "NoCopy.h"
#include "geometry/Point2LL.h"

namespace cura
{

class Point3D;

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

    struct ColorObject
    {
        bool is_enum_;
        Color color_;
        int r_, g_, b_;

        ColorObject(Color color)
            : is_enum_(true)
            , color_(color)
        {
        }

        ColorObject(int r, int g, int b)
            : is_enum_(false)
            , r_(r)
            , g_(g)
            , b_(b)
        {
        }
    };

private:
    std::string toString(const Color color) const;
    std::string toString(const ColorObject& color) const;
    void handleFlush(const bool flush) const;

    FILE* out_; // the output file
    const AABB aabb_; // the boundary box to display
    const Point2LL aabb_size_;
    const Point2LL canvas_size_;
    const double scale_;
    ColorObject background_;
    size_t layer_nr_ = 1;

    bool output_is_html_;

public:
    SVG(std::string filename, const AABB aabb, const Point2LL canvas_size = Point2LL(1024, 1024), const ColorObject background = Color::NONE);
    SVG(std::string filename, const AABB aabb, const double scale, const ColorObject background = Color::NONE);
    SVG(std::string filename, const AABB aabb, const double scale, const Point2LL canvas_size, const ColorObject background = Color::NONE);

    ~SVG();

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
    Point3D transformF(const Point2LL& p) const;

    void writeComment(const std::string& comment) const;

    void writeAreas(const Shape& polygons, const ColorObject color = Color::GRAY, const ColorObject outline_color = Color::BLACK, const double stroke_width = 1.0) const;

    void writeAreas(const Polygon& polygon, const ColorObject color = Color::GRAY, const ColorObject outline_color = Color::BLACK, const double stroke_width = 1.0) const;

    void writePoint(const Point2LL& p, const bool write_coords = false, const double size = 5.0, const ColorObject color = Color::BLACK) const;

    void writePoints(const Polygon& poly, const bool write_coords = false, const double size = 5.0, const ColorObject color = Color::BLACK) const;

    void writePoints(const Shape& polygons, const bool write_coords = false, const double size = 5.0, const ColorObject color = Color::BLACK) const;

    /*!
     * \brief Draws a polyline on the canvas.
     *
     * The polyline is the set of line segments between each pair of consecutive
     * points in the specified vector.
     *
     * \param polyline A set of points between which line segments must be
     * drawn.
     * \param color The colour of the line segments. If this is not specified,
     * black will be used.
     */
    void writeLines(const std::vector<Point2LL>& polyline, const ColorObject color = Color::BLACK) const;

    void writeLine(const Point2LL& a, const Point2LL& b, const ColorObject color = Color::BLACK, const double stroke_width = 1.0, const bool flush = true) const;

    void writeArrow(const Point2LL& a, const Point2LL& b, const ColorObject color = Color::BLACK, const double stroke_width = 1.0, const double head_size = 5.0) const;

    void writeLineRGB(const Point2LL& from, const Point2LL& to, const int r = 0, const int g = 0, const int b = 0, const double stroke_width = 1.0) const;

    /*!
     * \brief Draws a dashed line on the canvas from point A to point B.
     *
     * This is useful in the case where multiple lines may overlap each other.
     *
     * \param a The starting endpoint of the line.
     * \param b The ending endpoint of the line.
     * \param color The stroke colour of the line.
     */
    void writeDashedLine(const Point2LL& a, const Point2LL& b, ColorObject color = Color::BLACK) const;

    template<typename... Args>
    void printf(const char* txt, Args&&... args) const;

    void writeText(const Point2LL& p, const std::string& txt, const ColorObject color = Color::BLACK, const double font_size = 10.0) const;

    void writePolygons(const Shape& polys, const ColorObject color = Color::BLACK, const double stroke_width = 1.0, const bool flush = true) const;

    void writePolygon(Polygon poly, const ColorObject color = Color::BLACK, const double stroke_width = 1.0, const bool flush = true) const;

    void writePolylines(const Shape& polys, const ColorObject color = Color::BLACK, const double stroke_width = 1.0, const bool flush = true) const;

    void writePolyline(const Polygon& poly, const ColorObject color = Color::BLACK, const double stroke_width = 1.0) const;

    void writePolyline(const Polyline& poly, const ColorObject color = Color::BLACK, const double stroke_width = 1.0, const bool flush = true) const;

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
    void writeCoordinateGrid(const coord_t grid_size = MM2INT(1), const Color color = Color::BLACK, const double stroke_width = 0.1, const double font_size = 10.0) const;

    /*!
     * Draws the provided Voronoi diagram.
     *
     * @tparam T numeric type
     * @param voronoi The Voronoi diagram to draw.
     * @param color  The colour to draw the diagram with.
     * @param stroke_width The width of the lines.
     */
    template<typename T> // Currently our compiler for Mac can't handle `template<std::floating_point T>`, since aparently floating_point isn't in std yet.
    void writeVoronoiDiagram(const boost::polygon::voronoi_diagram<T>& voronoi_diagram, const Color color = Color::BLACK, const double stroke_width = 0.1) const
    {
        for (const auto& edge : voronoi_diagram.edges())
        {
            if (! edge.is_finite())
            {
                continue;
            }

            const auto& v0 = edge.vertex0();
            const auto& v1 = edge.vertex1();

            if (v0 == nullptr || v1 == nullptr)
            {
                continue;
            }

            writeLine(Point(v0->x(), v0->y()), Point(v1->x(), v1->y()), color, stroke_width);
        }
    }
};

template<typename... Args>
void SVG::printf(const char* txt, Args&&... args) const
{
    fprintf(out_, txt, args...);
}

} // namespace cura
#endif // SVG_H
