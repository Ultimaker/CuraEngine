#ifndef MERGE_INFILL_LINES_H
#define MERGE_INFILL_LINES_H

#include "utils/intpoint.h"
#include "gcodeExport.h"
#include "gcodePlanner.h"

namespace cura
{
    
class MergeInfillLines 
{
//     void merge(Point& from, Point& p0, Point& p1);
    GCodeExport& gcode; //!<  Where to write the combined line to
    std::vector<GCodePath>& paths; //!< The paths currently under consideration
    GCodePathConfig& travelConfig; //!< The travel settings used to see whether a path is a travel path or an extrusion path
    int64_t nozzle_size; //!< The diameter of the hole in the nozzle
    
    
    /*!
     * Whether the next two extrusion paths are convertible to a single line segment, starting from the end point the of the last travel move at \p path_idx_first_move
     * \param path_idx_first_move Index into MergeInfillLines::paths to the travel before the two extrusion moves udner consideration
     * \param first_middle Output parameter: the middle of the first extrusion move
     * \param second_middle Input/Output parameter: outputs the middle of the second extrusion move; inputs \p first_middle so we don't have to compute it
     * \param line_width Output parameter: The width of the resulting combined line (the average length of the lines combined)
     * \param use_second_middle_as_first Whether to use \p second_middle as input parameter for \p first_middle
     * \return Whether the next two extrusion paths are convertible to a single line segment, starting from the end point the of the last travel move at \p path_idx_first_move
     */
    bool isConvertible(unsigned int path_idx_first_move, Point& first_middle, Point& second_middle, int64_t& line_width, bool use_second_middle_as_first);
    
    /*!
     * Write an extrusion move with compensated width and compensated speed so that the material flow will be the same.
     * 
     * \param to The point to move to
     * \param speed The original speed
     * \param old_path The original path
     * \param new_line_width The width of the convewrted line (approximately the length of the original line)
     */
    void writeCompensatedMove(Point& to, double speed, GCodePath& old_path, int64_t new_line_width);
public:
    /*!
     * Simple constructor only used by MergeInfillLines::isConvertible to easily convey the environment
     */
    MergeInfillLines(GCodeExport& gcode, std::vector<GCodePath>& paths,  GCodePathConfig& travelConfig, int64_t nozzle_size) 
    : gcode(gcode), paths(paths), travelConfig(travelConfig), nozzle_size(nozzle_size) { }
    
    /*!
     * Check for lots of small moves and combine them into one large line.
     * Updates \p path_idx to the next path which is not combined.
     * 
     * \param gcode Where to write the combined line to
     * \param paths The paths currently under consideration
     * \param travelConfig The travel settings used to see whether a path is a travel path or an extrusion path
     * \param nozzle_size The diameter of the hole in the nozzle
     * \param speed A factor used to scale the movement speed
     * \param path_idx Input/Output parameter: The current index in \p paths where to start combining and the current index after combining as output parameter.
     * \return Whether lines have been merged and normal path-to-gcode generation can be skipped for the current resulting \p path_idx .
     */
    bool mergeInfillLines(double speed, unsigned int& path_idx);
    
};

}//namespace cura
#endif // MERGE_INFILL_LINES_H