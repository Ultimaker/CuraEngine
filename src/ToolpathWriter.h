//Copyright (c) 2019 Ultimaker B.V.


#ifndef TOOLPATH_WRITER_H
#define TOOLPATH_WRITER_H

#include <fstream>

#include "utils/ExtrusionLine.h"

namespace arachne
{

/*!
 * Write gcode for a single layer variable line width print
 */
class ToolpathWriter
{
public:
    ToolpathWriter(std::string filename);
    ~ToolpathWriter();
    
    void write(const std::vector<std::list<ExtrusionLine>>& result_polygons_per_index, const std::vector<std::list<ExtrusionLine>>& result_polylines_per_index);
private:
    std::ofstream file;
};




} // namespace arachne
#endif // TOOLPATH_WRITER_H
