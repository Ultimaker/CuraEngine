//Copyright (c) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <sstream>

#include "floatpoint.h"
#include "logoutput.h"
#include "polygon.h"
#include "SVG.h"

namespace cura {



std::string SVG::toString(Color color)
{
    switch (color)
    {
        case SVG::Color::BLACK: return "black";
        case SVG::Color::WHITE: return "white";
        case SVG::Color::GRAY: return "gray";
        case SVG::Color::RED: return "red";
        case SVG::Color::BLUE: return "blue";
        case SVG::Color::GREEN: return "green";
        case SVG::Color::ORANGE: return "orange";
        case SVG::Color::MAGENTA: return "magenta";
        case SVG::Color::YELLOW: return "yellow";
        case SVG::Color::NONE: return "none";
        default: return "black";
    }
}

std::string SVG::toString(ColorObject& color)
{
    if (color.is_enum) return toString(color.color);
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

SVG::SVG(std::string filename, AABB aabb, double scale, ColorObject background)
: SVG(filename, aabb, scale, (aabb.max - aabb.min) * scale, background)
{
}

SVG::SVG(std::string filename, AABB aabb, double scale, Point canvas_size, ColorObject background)
: aabb(aabb)
, aabb_size(aabb.max - aabb.min)
, canvas_size(canvas_size)
, scale(scale)
, background(background)
{
    output_is_html = strcmp(filename.c_str() + strlen(filename.c_str()) - 4, "html") == 0;
    out = fopen(filename.c_str(), "w");
    if(!out)
    {
        logError("The file %s could not be opened for writing.",filename.c_str());
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
    fprintf(out,"   xmlns=\"http://www.w3.org/2000/svg\"\n");
    fprintf(out,"   xmlns:inkscape=\"http://www.inkscape.org/namespaces/inkscape\"\n");
    fprintf(out,"   height=\"%f\"\n", scale * (aabb.max.Y - aabb.min.Y));
    fprintf(out,"   width=\"%f\"\n", scale * (aabb.max.X - aabb.min.X));
    fprintf(out,"   version=\"1.1\">\n");
    fprintf(out,"  <g\n");
    fprintf(out,"    inkscape:groupmode=\"layer\"\n");
    fprintf(out,"    inkscape:label=\"layer%zu\"\n", layer_nr );
    fprintf(out,"    id=\"layer%zu\">\n", layer_nr );

    if (!background.is_enum || background.color != Color::NONE)
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
    fprintf(out,"  <g\n");
    fprintf(out,"    inkscape:groupmode=\"layer\"\n");
    fprintf(out,"    inkscape:label=\"layer%zu\"\n", layer_nr );
    fprintf(out,"    id=\"layer%zu\">\n", layer_nr );
}

Point SVG::transform(const Point& p) 
{
    return Point((p.X - aabb.min.X) * scale, (p.Y - aabb.min.Y) * scale);
}

FPoint3 SVG::transformF(const Point& p) 
{
    return FPoint3((p.X - aabb.min.X) * scale, (p.Y-aabb.min.Y) * scale, 0.0);
}

void SVG::writeComment(std::string comment)
{
    fprintf(out, "<!-- %s -->\n", comment.c_str());
}

void SVG::writeAreas(const Polygons& polygons, ColorObject color, ColorObject outline_color, float stroke_width) 
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

void SVG::writeAreas(ConstPolygonRef polygon, ColorObject color, ColorObject outline_color, float stroke_width)
{
    fprintf(out,"<polygon fill=\"%s\" stroke=\"%s\" stroke-width=\"%f\" points=\"",toString(color).c_str(),toString(outline_color).c_str(), stroke_width); //The beginning of the polygon tag.
    for (const Point& point : polygon) //Add every point to the list of points.
    {
        FPoint3 transformed = transformF(point);
        fprintf(out,"%f,%f ",transformed.x,transformed.y);
    }
    fprintf(out,"\" />\n"); //The end of the polygon tag.
}

void SVG::writePoint(const Point& p, bool write_coords, int size, ColorObject color)
{
    FPoint3 pf = transformF(p);
    fprintf(out, "<circle cx=\"%f\" cy=\"%f\" r=\"%d\" stroke=\"%s\" stroke-width=\"1\" fill=\"%s\" />\n",pf.x, pf.y, size, toString(color).c_str(), toString(color).c_str());
    
    if (write_coords)
    {
        fprintf(out, "<text x=\"%f\" y=\"%f\" style=\"font-size: 10px;\" fill=\"black\">%lli,%lli</text>\n",pf.x, pf.y, p.X, p.Y);
    }
}

void SVG::writePoints(ConstPolygonRef poly, bool write_coords, int size, ColorObject color)
{
    for (const Point& p : poly)
    {
        writePoint(p, write_coords, size, color);
    }
}

void SVG::writePoints(Polygons& polygons, bool write_coords, int size, ColorObject color)
{
    for (PolygonRef poly : polygons)
    {
        writePoints(poly, write_coords, size, color);
    }
}

void SVG::writeLines(std::vector<Point> polyline, ColorObject color)
{
    if(polyline.size() <= 1) //Need at least 2 points.
    {
        return;
    }
    
    FPoint3 transformed = transformF(polyline[0]); //Element 0 must exist due to the check above.
    fprintf(out,"<path fill=\"none\" stroke=\"%s\" stroke-width=\"1\" d=\"M%f,%f",toString(color).c_str(), transformed.x, transformed.y); //Write the start of the path tag and the first endpoint.
    for(size_t point = 1;point < polyline.size();point++)
    {
        transformed = transformF(polyline[point]);
        fprintf(out,"L%f,%f", transformed.x, transformed.y); //Write a line segment to the next point.
    }
    fprintf(out,"\" />\n"); //Write the end of the tag.
}

void SVG::writeLine(const Point& a, const Point& b, ColorObject color, float stroke_width)
{
    FPoint3 fa = transformF(a);
    FPoint3 fb = transformF(b);
    fprintf(out, "<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" style=\"stroke:%s;stroke-width:%f\" />\n", fa.x, fa.y, fb.x, fb.y, toString(color).c_str(), stroke_width);
}

void SVG::writeArrow(const Point& a, const Point& b, ColorObject color, float stroke_width, int rel_head_size_divisor, coord_t offset)
{
    Point ab = b - a;
    Point nd = turn90CCW(ab) / rel_head_size_divisor / 2;
    Point n = normal(turn90CCW(ab), offset);
    Point d = ab / rel_head_size_divisor / 2;
    writeLine(a + n, b + n - d, color, stroke_width);
    writeLine(b + n - d, b + n + nd - d * 3, color, stroke_width);
}

void SVG::writeLineRGB(const Point& from, const Point& to, int r, int g, int b, float stroke_width)
{
    FPoint3 fa = transformF(from);
    FPoint3 fb = transformF(to);
    fprintf(out, "<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" style=\"stroke:rgb(%i,%i,%i);stroke-width:%f\" />\n", fa.x, fa.y, fb.x, fb.y, r, g, b, stroke_width);
}

void SVG::writeDashedLine(const Point& a, const Point& b, ColorObject color)
{
    FPoint3 fa = transformF(a);
    FPoint3 fb = transformF(b);
    fprintf(out,"<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" stroke=\"%s\" stroke-width=\"1\" stroke-dasharray=\"5,5\" />\n",fa.x,fa.y,fb.x,fb.y,toString(color).c_str());
}

void SVG::writeText(Point p, std::string txt, ColorObject color, coord_t font_size)
{
    FPoint3 pf = transformF(p);
    fprintf(out, "<text x=\"%f\" y=\"%f\" style=\"font-size: %llipx;\" fill=\"%s\">%s</text>\n",pf.x, pf.y, font_size, toString(color).c_str(), txt.c_str());
}

void SVG::writePolygons(const Polygons& polys, ColorObject color, float stroke_width)
{
    for (ConstPolygonRef poly : polys)
    {
        writePolygon(poly, color, stroke_width);
    }
}

void SVG::writePolygon(ConstPolygonRef poly, ColorObject color, float stroke_width)
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


void SVG::writePolylines(const Polygons& polys, ColorObject color, float stroke_width)
{
    for (ConstPolygonRef poly : polys)
    {
        writePolyline(poly, color, stroke_width);
    }
}

void SVG::writePolyline(ConstPolygonRef poly, ColorObject color, float stroke_width)
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
            if (g > 255) g = 255 * 2 - g;
            int b = (i * 255 * 5 / size) % (255 * 2);
            if (b > 255) b = 255 * 2 - b;
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

} // namespace cura
