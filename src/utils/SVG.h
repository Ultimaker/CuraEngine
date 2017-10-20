#ifndef SVG_H
#define SVG_H

#include <stdio.h> // for file output

#include "polygon.h"
#include "intpoint.h"
#include "AABB.h"
#include "logoutput.h"
#include "NoCopy.h"

namespace cura {

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
        YELLOW
    };
    
private:
    
    std::string toString(Color color)
    {
        switch (color)
        {
            case SVG::Color::BLACK: return "black";
            case SVG::Color::WHITE: return "white";
            case SVG::Color::GRAY: return "gray";
            case SVG::Color::RED: return "red";
            case SVG::Color::BLUE: return "blue";
            case SVG::Color::GREEN: return "green";
            case SVG::Color::YELLOW: return "yellow";
            default: return "black";
        }
    }
    
    
    
    FILE* out; // the output file
    const AABB aabb; // the boundary box to display
    const Point aabb_size;
    const Point border;
    const Point canvas_size;
    const double scale;

public:
    SVG(const char* filename, AABB aabb, Point canvas_size = Point(1024, 1024))
    : aabb(aabb)
    , aabb_size(aabb.max - aabb.min)
    , border(200,100)
    , canvas_size(canvas_size)
    , scale(std::min(double(canvas_size.X - border.X * 2) / aabb_size.X, double(canvas_size.Y - border.Y * 2) / aabb_size.Y))
    {
        out = fopen(filename, "w");
        if(!out)
        {
            logError("The file %s could not be opened for writing.",filename);
        }
        fprintf(out, "<!DOCTYPE html><html><body>\n");
        fprintf(out, "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" style=\"width:%llipx;height:%llipx\">\n", canvas_size.X, canvas_size.Y);
    }

    ~SVG()
    {
        fprintf(out, "</svg>\n");
        fprintf(out, "</body></html>");
        fclose(out);
    }

    /*!
     * transform a point in real space to canvas space
     */
    Point transform(const Point& p) 
    {
        return Point((p.X-aabb.min.X)*scale, canvas_size.X - border.X - (p.Y-aabb.min.Y)*scale) + border;
    }

    void writeComment(std::string comment)
    {
        fprintf(out, "<!-- %s -->\n", comment.c_str());
    }

    void writeAreas(const Polygons& polygons, Color color = Color::GRAY, Color outline_color = Color::BLACK) 
    {
        for(PolygonsPart& parts : polygons.splitIntoParts())
        {
            for(unsigned int j=0;j<parts.size();j++)
            {
                fprintf(out, "<polygon points=\"");
                for (Point& p : parts[j])
                {
                    Point fp = transform(p);
                    fprintf(out, "%lli,%lli ", fp.X, fp.Y);
                }
                if (j == 0)
                    fprintf(out, "\" style=\"fill:%s;stroke:%s;stroke-width:1\" />\n", toString(color).c_str(), toString(outline_color).c_str());
                else
                    fprintf(out, "\" style=\"fill:white;stroke:%s;stroke-width:1\" />\n", toString(outline_color).c_str());
            }
        }
    }

    void writeAreas(std::vector<Point> polygon,Color color = Color::GRAY,Color outline_color = Color::BLACK)
    {
        fprintf(out,"<polygon fill=\"%s\" stroke=\"%s\" stroke-width=\"1\" points=\"",toString(color).c_str(),toString(outline_color).c_str()); //The beginning of the polygon tag.
        for(Point& point : polygon) //Add every point to the list of points.
        {
            Point transformed = transform(point);
            fprintf(out,"%lli,%lli ",transformed.X,transformed.Y);
        }
        fprintf(out,"\" />\n"); //The end of the polygon tag.
    }

    void writePoint(const Point& p, bool write_coords=false, int size = 5, Color color = Color::BLACK)
    {
        Point pf = transform(p);
        fprintf(out, "<circle cx=\"%lli\" cy=\"%lli\" r=\"%d\" stroke=\"%s\" stroke-width=\"1\" fill=\"%s\" />\n",pf.X, pf.Y, size, toString(color).c_str(), toString(color).c_str());
        
        if (write_coords)
        {
            fprintf(out, "<text x=\"%lli\" y=\"%lli\" style=\"font-size: 10px;\" fill=\"black\">%lli,%lli</text>\n",pf.X, pf.Y, p.X, p.Y);
        }
    }

    void writePoints(ConstPolygonRef poly, bool write_coords=false, int size = 5, Color color = Color::BLACK)
    {
        for (const Point& p : poly)
        {
            writePoint(p, write_coords, size, color);
        }
    }

    void writePoints(Polygons& polygons, bool write_coords=false, int size = 5, Color color = Color::BLACK)
    {
        for (PolygonRef poly : polygons)
        {
            writePoints(poly, write_coords, size, color);
        }
    }

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
    void writeLines(std::vector<Point> polyline, Color color = Color::BLACK)
    {
        if(polyline.size() <= 1) //Need at least 2 points.
        {
            return;
        }
        
        Point transformed = transform(polyline[0]); //Element 0 must exist due to the check above.
        fprintf(out,"<path fill=\"none\" stroke=\"%s\" stroke-width=\"1\" d=\"M%lli,%lli",toString(color).c_str(),transformed.X,transformed.Y); //Write the start of the path tag and the first endpoint.
        for(size_t point = 1;point < polyline.size();point++)
        {
            transformed = transform(polyline[point]);
            fprintf(out,"L%lli,%lli",transformed.X,transformed.Y); //Write a line segment to the next point.
        }
        fprintf(out,"\" />\n"); //Write the end of the tag.
    }

    void writeLine(const Point& a, const Point& b, Color color = Color::BLACK, int stroke_width = 1)
    {
        Point fa = transform(a);
        Point fb = transform(b);
        fprintf(out, "<line x1=\"%lli\" y1=\"%lli\" x2=\"%lli\" y2=\"%lli\" style=\"stroke:%s;stroke-width:%i\" />\n", fa.X, fa.Y, fb.X, fb.Y, toString(color).c_str(), stroke_width);
    }
    
    /*!
     * \brief Draws a dashed line on the canvas from point A to point B.
     * 
     * This is useful in the case where multiple lines may overlap each other.
     * 
     * \param a The starting endpoint of the line.
     * \param b The ending endpoint of the line.
     * \param color The stroke colour of the line.
     */
    void writeDashedLine(const Point& a,const Point& b,Color color = Color::BLACK)
    {
        Point fa = transform(a);
        Point fb = transform(b);
        fprintf(out,"<line x1=\"%lli\" y1=\"%lli\" x2=\"%lli\" y2=\"%lli\" stroke=\"%s\" stroke-width=\"1\" stroke-dasharray=\"5,5\" />\n",fa.X,fa.Y,fb.X,fb.Y,toString(color).c_str());
    }

    template<typename... Args>
    void printf(const char* txt, Args&&... args)
    {
        fprintf(out, txt, args...);
    }
    void writeText(Point p, std::string txt, Color color = Color::BLACK, coord_t font_size = 10)
    {
        Point pf = transform(p);
        fprintf(out, "<text x=\"%lli\" y=\"%lli\" style=\"font-size: %llipx;\" fill=\"%s\">%s</text>\n",pf.X, pf.Y, font_size, toString(color).c_str(), txt.c_str());
    }
    void writePolygons(const Polygons& polys, Color color = Color::BLACK, int stroke_width = 1)
    {
        for (ConstPolygonRef poly : polys)
        {
            writePolygon(poly, color, stroke_width);
        }
    }
    void writePolygon(ConstPolygonRef poly, Color color = Color::BLACK, int stroke_width = 1)
    {
        if (poly.size() == 0)
        {
            return;
        }
        Point p0 = poly.back();
        for (Point p1 : poly)
        {
            writeLine(p0, p1, color, stroke_width);
            p0 = p1;
        }
    }
    
    
    
    
    /*
    void Polygons::debugOutputHTML(const char* filename, bool dotTheVertices)
{
    FILE* out = fopen(filename, "w");
    fprintf(out, "<!DOCTYPE html><html><body>");
    Point modelSize = max() - min();
    modelSize.X = std::max(modelSize.X, modelSize.Y);
    modelSize.Y = std::max(modelSize.X, modelSize.Y);
    Point modelMin = min();

    fprintf(out, "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" style=\"width: 500px; height:500px\">\n");
    for(PolygonsPart& parts : splitIntoParts())
    {
        for(unsigned int j=0;j<parts.size();j++)
        {
            Polygon poly = parts[j];
            fprintf(out, "<polygon points=\"");
            for(Point& p : poly)
            {
                fprintf(out, "%f,%f ", float(p.X - modelMin.X)/modelSize.X*500, float(p.Y - modelMin.Y)/modelSize.Y*500);
            }
            if (j == 0)
                fprintf(out, "\" style=\"fill:gray; stroke:black;stroke-width:1\" />\n");
            else
                fprintf(out, "\" style=\"fill:red; stroke:black;stroke-width:1\" />\n");
            
            if (dotTheVertices)
                for(Point& p : poly)
                    fprintf(out, "<circle cx=\"%f\" cy=\"%f\" r=\"2\" stroke=\"black\" stroke-width=\"3\" fill=\"black\" />", float(p.X - modelMin.X)/modelSize.X*500, float(p.Y - modelMin.Y)/modelSize.Y*500);
        }
    }
    fprintf(out, "</svg>\n");
    fprintf(out, "</body></html>");
    fclose(out);
    } */   

};

} // namespace cura
#endif // SVG_H
