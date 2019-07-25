//Copyright (c) 2019 Ultimaker B.V.


#ifndef TEST_GEOMETRY_SVG_LOADER_H
#define TEST_GEOMETRY_SVG_LOADER_H

#include <fstream>
#include <regex>

#include "utils/polygon.h"
#include "utils/floatpoint.h"
#include "utils/logoutput.h"

namespace arachne
{

class SVGloader
{
public:
    static Polygons load(std::string filename)
    {
                
        Polygons ret;
        
        std::ifstream file;
        try
        {
            file.open(filename);
        }
        catch (const std::exception& e)
        {
            logError("Couldn't open file '%s' for reading svg.\n", filename.c_str());
            std::cerr << std::flush;
            std::exit(-1);
            return ret;
        }
        if (!file.is_open())
        {
            logError("Couldn't open file '%s' for reading svg.\n", filename.c_str());
            std::exit(-1);
            return ret;
        }
        std::regex poly_regex("<(polygon)|(path) .*/>");

        std::regex points_regex(" points=\"([^\"]*)\"");
        std::smatch points_sm;

        std::regex d_regex("d=\"([^\"]*)\"");
        std::smatch d_sm;

        std::regex point_regex("([^ ]+)");
        std::smatch point_sm;

        std::string line;
        while (getline(file, line))
        {
            if (std::regex_search(line, points_sm, points_regex))
            {
                if (points_sm.size() < 1)
                {
                    assert(false);
                    continue; // didn't contain points
                }
                std::string points_str = points_sm[1];
                        
                std::transform(points_str.begin(), points_str.end(), points_str.begin(),
                    [](unsigned char c){ return std::tolower(c); });

                PolygonRef poly = ret.newPoly();
                PolygonPointer poly_p(poly);
                bool additive = false;
                bool painting = true;
                bool is_vertical = false;
                bool is_horizontal = false;
                Point current(0,0);
                while (std::regex_search (points_str, point_sm, point_regex))
                {
                    float x, y;
                    std::string point_str = point_sm[0];
                    if (point_str.compare("") == 0) break;
                    if (std::sscanf(point_str.c_str(), "%f,%f", &x, &y) == 2)
                    {
                        Point here(MM2INT(x), MM2INT(y));
                        current = additive? current + here : here;
                        poly_p->emplace_back(current);
                    }
                    else
                    {
                        std::cerr << "Couldn't parse '" << point_str << "'\n";
                    }
                    points_str = point_sm.suffix().str();
                }
            }
        }
    }
};

} // namespace arachne
#endif // TEST_GEOMETRY_SVG_LOADER_H
