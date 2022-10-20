//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SVG_H
#define SVG_H

#include <stdio.h> // for file output

#include "AABB.h"
#include "ExtrusionLine.h" //To accept variable-width paths.
#include "IntPoint.h"
#include "NoCopy.h"

namespace cura
{

class FPoint3;

class SVG : NoCopy
{
public:
    enum class Color {
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
        bool is_enum;
        Color color;
        int r, g, b;
        ColorObject(Color color)
        : is_enum(true)
        , color(color)
        {}
        ColorObject(int r, int g, int b)
        : is_enum(false)
        , r(r)
        , g(g)
        , b(b)
        {}
    };
private:

    std::string toString(const Color color) const;
    std::string toString(const ColorObject& color) const;

    FILE* out; // the output file
    const AABB aabb; // the boundary box to display
    const Point aabb_size;
    const Point canvas_size;
    const double scale;
    ColorObject background;
    size_t layer_nr = 1;

    bool output_is_html;

public:
    SVG(std::string filename, const AABB aabb, const Point canvas_size = Point(1024, 1024), const ColorObject background = Color::NONE);
    SVG(std::string filename, const AABB aabb, const double scale, const ColorObject background = Color::NONE);
    SVG(std::string filename, const AABB aabb, const double scale, const Point canvas_size, const ColorObject background = Color::NONE);

    ~SVG();

    /*!
     * get the scaling factor applied to convert real space to canvas space
     */
    double getScale() const;

    void nextLayer();

    /*!
     * transform a point in real space to canvas space
     */
    Point transform(const Point& p) const;

    /*!
     * transform a point in real space to canvas space with more precision
     */
    FPoint3 transformF(const Point& p) const;

    void writeComment(const std::string& comment) const;

    void writeAreas(const Polygons& polygons, const ColorObject color = Color::GRAY, const ColorObject outline_color = Color::BLACK, const float stroke_width = 1) const;

    void writeAreas(ConstPolygonRef polygon, const ColorObject color = Color::GRAY, const ColorObject outline_color = Color::BLACK, const float stroke_width = 1) const;

    void writePoint(const Point& p, const bool write_coords = false, const float size = 5.0, const ColorObject color = Color::BLACK) const;

    void writePoints(ConstPolygonRef poly, const bool write_coords = false, const float size = 5.0, const ColorObject color = Color::BLACK) const;

    void writePoints(const Polygons& polygons, const bool write_coords = false, const float size = 5.0, const ColorObject color = Color::BLACK) const;

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
    void writeLines(const std::vector<Point>& polyline, const ColorObject color = Color::BLACK) const;

    void writeLine(const Point& a, const Point& b, const ColorObject color = Color::BLACK, const float stroke_width = 1) const;

    void writeArrow(const Point& a, const Point& b, const ColorObject color = Color::BLACK, const float stroke_width = 1, const float head_size = 5.0) const;

    void writeLineRGB(const Point& from, const Point& to, const int r = 0, const int g = 0, const int b = 0, const float stroke_width = 1) const;

    /*!
     * \brief Draws a dashed line on the canvas from point A to point B.
     * 
     * This is useful in the case where multiple lines may overlap each other.
     * 
     * \param a The starting endpoint of the line.
     * \param b The ending endpoint of the line.
     * \param color The stroke colour of the line.
     */
    void writeDashedLine(const Point& a,const Point& b, ColorObject color = Color::BLACK) const;

    template<typename... Args>
    void printf(const char* txt, Args&&... args) const;

    void writeText(const Point& p, const std::string& txt, const ColorObject color = Color::BLACK, const float font_size = 10) const;

    void writePolygons(const Polygons& polys, const ColorObject color = Color::BLACK, const float stroke_width = 1) const;

    void writePolygon(ConstPolygonRef poly, const ColorObject color = Color::BLACK, const float stroke_width = 1) const;

    void writePolylines(const Polygons& polys, const ColorObject color = Color::BLACK, const float stroke_width = 1) const;

    void writePolyline(ConstPolygonRef poly, const ColorObject color = Color::BLACK, const float stroke_width = 1) const;

    /*!
     * Draw variable-width paths into the image.
     *
     * The paths are drawn with the correct line width, as given in the paths,
     * but there is a multiplicative factor to adjust the width with.
     * \param paths The paths to draw.
     * \param color The color to draw the paths with.
     * \param width_factor A multiplicative factor on the line widths.
     */
    void writePaths(const std::vector<VariableWidthLines>& paths, const ColorObject color = Color::BLACK, const float width_factor = 1.0) const;

    /*!
     * Draw variable-width lines into the image.
     *
     * The lines are drawn with the correct line width, as given in the lines,
     * but there is a multiplicative factor to adjust the width with.
     * \param lines The lines to draw.
     * \param color The color to draw the lines with.
     * \param width_factor A multiplicative factor on the line widths.
     */
    void writeLines(const VariableWidthLines& lines, const ColorObject color = Color::BLACK, const float width_factor = 1.0) const;

    /*!
     * Draw a variable-width line into the image.
     *
     * The line is drawn with the correct line width, as given in the junctions,
     * but there is a multiplicative factor to adjust the width with.
     * \param line The line to draw.
     * \param color The color to draw the line with.
     * \param width_factor A multiplicative factor on the line width.
     */
    void writeLine(const ExtrusionLine& line, const ColorObject color = Color::BLACK, const float width_factor = 1.0) const;

    /*!
     * Draws a grid across the image and writes down coordinates.
     *
     * Coordinates are always written in millimeters.
     * \param grid_size Size of the grid cells.
     * \param color The colour to draw the grid with.
     * \param stroke_width The width of the grid lines.
     * \param font_size The size of the font to write the coordinates with.
     */
    void writeCoordinateGrid(const coord_t grid_size = MM2INT(1), const Color color = Color::BLACK, const float stroke_width = 0.1, const float font_size = 10) const;

};

template<typename... Args>
void SVG::printf(const char* txt, Args&&... args) const
{
    fprintf(out, txt, args...);
}

} // namespace cura
#endif // SVG_H
