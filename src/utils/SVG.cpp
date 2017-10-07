/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "SVG.h"


namespace cura {

void SVG::writeLineRGB(const Point& from, const Point& to, int r, int g, int b, int stroke_width)
{
    Point fa = transform(from);
    Point fb = transform(to);
    fprintf(out, "<line x1=\"%lli\" y1=\"%lli\" x2=\"%lli\" y2=\"%lli\" style=\"stroke:rgb(%i,%i,%i);stroke-width:%i\" />\n", fa.X, fa.Y, fb.X, fb.Y, r, g, b, stroke_width);
}

void SVG::writePolygon(ConstPolygonRef poly, Color color, int stroke_width)
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
        if (color == Color::RAINBOW)
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
