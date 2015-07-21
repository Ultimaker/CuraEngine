#include "MergeInfillLines.h"

namespace cura
{
bool MergeInfillLines::mergeInfillLines(GCodeExport& gcode, std::vector<GCodePath>& paths, GCodePathConfig& travelConfig, int64_t nozzle_size, double speed, unsigned int& path_idx)
{ //Check for lots of small moves and combine them into one large line
    Point prev_middle;
    Point last_middle;
    int64_t line_width;
    
    MergeInfillLines merger(gcode, paths, travelConfig, nozzle_size);
    
    if (merger.isConvertible(path_idx, prev_middle, last_middle, line_width, false))
    {
        //   path_idx + 3 is the index of the second extrusion move to be converted in combination with the first
        {
            GCodePath& last_path = paths[path_idx + 3];
                                        // add first lil path of half line width
            gcode.writeMove(prev_middle - normal(last_middle-prev_middle, last_path.config->getLineWidth() / 2), travelConfig.getSpeed(), 0);
            gcode.writeMove(last_middle, speed * line_width / last_path.config->getLineWidth(), last_path.getExtrusionMM3perMM() * line_width / last_path.config->getLineWidth());
        }
        
        path_idx += 2;
        for (; merger.isConvertible(path_idx, prev_middle, last_middle, line_width, true); path_idx += 2)
        {
            GCodePath& last_path = paths[path_idx + 3];
            gcode.writeMove(last_middle, speed * line_width / last_path.config->getLineWidth(), last_path.getExtrusionMM3perMM() * line_width / last_path.config->getLineWidth());
        }
        // add last lil path of half line width
        GCodePath& last_path = paths[path_idx + 1];
        gcode.writeMove(last_middle + normal(last_middle-prev_middle, last_path.config->getLineWidth() / 2), speed * line_width / last_path.config->getLineWidth(), last_path.getExtrusionMM3perMM() * line_width / last_path.config->getLineWidth());
        path_idx = path_idx + 1; // means that the next path considered is the travel path after the converted extrusion path corresponding to the updated path_idx
        return true;
    }
    return false;
};

bool MergeInfillLines::isConvertible(unsigned int path_idx_first_move, Point& first_middle, Point& second_middle, int64_t& line_width, bool use_second_middle_as_first)
{
    int64_t max_line_width = nozzle_size * 3 / 2;
    
    
    unsigned int idx = path_idx_first_move;
    if (idx + 3 > paths.size()-1) return false;
    if (paths[idx+0].config != &travelConfig) return false;
    if (paths[idx+1].points.size() > 1) return false;
    if (paths[idx+1].config == &travelConfig) return false;
//                 if (paths[idx+2].points.size() > 1) return false;
    if (paths[idx+2].config != &travelConfig) return false;
    if (paths[idx+3].points.size() > 1) return false;
    if (paths[idx+3].config == &travelConfig) return false;
    
    Point& a = paths[idx+0].points.back(); // first extruded line from
    Point& b = paths[idx+1].points.back(); // first extruded line to
    Point& c = paths[idx+2].points.back(); // second extruded line from
    Point& d = paths[idx+3].points.back(); // second extruded line to
    Point cd = d - c;
    Point ab = b - a;
    
    int64_t prod = dot(ab,cd);
    if (prod*prod + 20 < vSize2(ab) * vSize2(cd)) return false; // extrusion moves not in the same or opposite diraction
    if (prod < 0) { ab = ab * -1; }
    
    
    Point infill_vector = (cd + ab) / 2;
    
    if (!shorterThen(infill_vector, 5 * nozzle_size)) return false; // infill lines too far apart
                    
    first_middle = (use_second_middle_as_first)?
                    second_middle :
                    (a + b) / 2;
    second_middle = (c + d) / 2;
    
    Point dir_vector_perp = crossZ(second_middle - first_middle);
    int64_t dir_vector_perp_length = vSize(dir_vector_perp); // == dir_vector_length
    if (dir_vector_perp_length == 0) return false;
    if (dir_vector_perp_length > 5 * nozzle_size) return false; // infill lines too far apart
    
    
    line_width = std::abs( dot(dir_vector_perp, infill_vector) / dir_vector_perp_length );
    if (line_width > max_line_width) return false; // combined lines would be too wide
    
    { // check whether the two lines are adjacent
        Point ca = first_middle - c;
        double ca_size = vSizeMM(ca);
        double cd_size = vSizeMM(cd);
        double prod = INT2MM(dot(ca, cd));
        double fraction = prod / ( ca_size * cd_size );
        int64_t line2line_dist = MM2INT(cd_size * std::sqrt(1.0 - fraction * fraction));
        
        if (line2line_dist + 20 > paths[idx+1].config->getLineWidth()) return false; // there is a gap between the two lines
    }
    
    return true;
};

     /*   
void MergeInfillLines::merge(Point& from, Point& p0, Point& p1)
{ //Check for lots of small moves and combine them into one large line
    if (path->points.size() == 1 && path->config != &travelConfig); // && shorterThen(from - path->points[0], path->config->getLineWidth() * 2))
    {
        Point p0 = path->points[0];
        unsigned int path_idx_last = path_idx + 1; // index of the last short move 
        while(path_idx_last < paths.size() && paths[path_idx_last].points.size() == 1 && shorterThen(p0 - paths[path_idx_last].points[0], path->config->getLineWidth() * 2))
        {
            p0 = paths[path_idx_last].points[0];
            path_idx_last ++;
        }
        if (paths[path_idx_last-1].config == &travelConfig)
            path_idx_last --;
        
        if (path_idx_last > path_idx + 2)
        {
            p0 = from;
            for(unsigned int path_idx_short = path_idx; path_idx_short < path_idx_last-1; path_idx_short+=2)
            {
                int64_t oldLen = vSize(p0 - paths[path_idx_short].points[0]);
                Point newPoint = (paths[path_idx_short].points[0] + paths[path_idx_short+1].points[0]) / 2;
                int64_t newLen = vSize(from - newPoint);
                if (newLen > 0)
                {
                    if (oldLen > 0)
                        gcode.writeMove(newPoint, speed * oldLen / newLen, path->getExtrusionMM3perMM() * newLen / oldLen);
                    else 
                        gcode.writeMove(newPoint, speed, path->getExtrusionMM3perMM());
                }
            }
            gcode.writeMove(paths[path_idx_last-1].points[0], speed, path->getExtrusionMM3perMM());
            path_idx = path_idx_last - 1;
            continue;
        }
    }
}*/
     
     
     
} // namespace cura