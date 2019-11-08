//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SVG_H
#define SVG_H

#include <stdio.h> // for file output

#include "AABB.h"
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

    std::string toString(Color color);
    std::string toString(ColorObject& color);

    FILE* out; // the output file
    const AABB aabb; // the boundary box to display
    const Point aabb_size;
    const Point canvas_size;
    const double scale;
    ColorObject background;
    size_t layer_nr = 1;

    bool output_is_html;

public:
    SVG(std::string filename, AABB aabb, Point canvas_size = Point(1024, 1024), ColorObject background = Color::NONE);
    SVG(std::string filename, AABB aabb, double scale, ColorObject background = Color::NONE);
    SVG(std::string filename, AABB aabb, double scale, Point canvas_size, ColorObject background = Color::NONE);

    ~SVG();

    /*!
     * get the scaling factor applied to convert real space to canvas space
     */
    double getScale() const;

    void nextLayer();

    /*!
     * transform a point in real space to canvas space
     */
    Point transform(const Point& p);

    /*!
     * transform a point in real space to canvas space with more precision
     */
    FPoint3 transformF(const Point& p);

    void writeComment(std::string comment);

    void writeAreas(const Polygons& polygons, ColorObject color = Color::GRAY, ColorObject outline_color = Color::BLACK, float stroke_width = 1);

    void writeAreas(ConstPolygonRef polygon, ColorObject color = Color::GRAY, ColorObject outline_color = Color::BLACK, float stroke_width = 1);

    void writePoint(const Point& p, bool write_coords=false, int size = 5, ColorObject color = Color::BLACK);

    void writePoints(ConstPolygonRef poly, bool write_coords=false, int size = 5, ColorObject color = Color::BLACK);

    void writePoints(Polygons& polygons, bool write_coords=false, int size = 5, ColorObject color = Color::BLACK);

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
    void writeLines(std::vector<Point> polyline, ColorObject color = Color::BLACK);

    void writeLine(const Point& a, const Point& b, ColorObject color = Color::BLACK, float stroke_width = 1);

    void writeArrow(const Point& a, const Point& b, ColorObject color = Color::BLACK, float stroke_width = 1, int rel_head_size_divisor = 20, coord_t offset = 20);

    void writeLineRGB(const Point& from, const Point& to, int r = 0, int g = 0, int b = 0, float stroke_width = 1);

    /*!
     * \brief Draws a dashed line on the canvas from point A to point B.
     * 
     * This is useful in the case where multiple lines may overlap each other.
     * 
     * \param a The starting endpoint of the line.
     * \param b The ending endpoint of the line.
     * \param color The stroke colour of the line.
     */
    void writeDashedLine(const Point& a,const Point& b, ColorObject color = Color::BLACK);

    template<typename... Args>
    void printf(const char* txt, Args&&... args);

    void writeText(Point p, std::string txt, ColorObject color = Color::BLACK, coord_t font_size = 10);

    void writePolygons(const Polygons& polys, ColorObject color = Color::BLACK, float stroke_width = 1);

    void writePolygon(ConstPolygonRef poly, ColorObject color = Color::BLACK, float stroke_width = 1);

    void writePolylines(const Polygons& polys, ColorObject color = Color::BLACK, float stroke_width = 1);

    void writePolyline(ConstPolygonRef poly, ColorObject color = Color::BLACK, float stroke_width = 1);

};

template<typename... Args>
void SVG::printf(const char* txt, Args&&... args)
{
    fprintf(out, txt, args...);
}

} // namespace cura
#endif // SVG_H
