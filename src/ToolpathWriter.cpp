//Copyright (c) 2019 Ultimaker B.V.

#include <cassert>

#include "ToolpathWriter.h"

namespace arachne
{

ToolpathWriter::ToolpathWriter(std::string filename)
: file(filename.c_str())
{
    if (!file.good())
        std::cerr << "ERROR: Cannot open '" << filename << "' for writing!\n";
    assert(file.good());
}

ToolpathWriter::~ToolpathWriter()
{
    file.close();
}

void ToolpathWriter::write(const std::vector<std::list<ExtrusionLine>>& result_polygons_per_index, const std::vector<std::list<ExtrusionLine>>& result_polylines_per_index)
{
    std::string type = "closed";
    for (const std::vector<std::list<ExtrusionLine>>& polys_per_index : {result_polygons_per_index, result_polylines_per_index})
    {
        for (size_t idx = 0; idx < polys_per_index.size(); idx++)
        {
            for (const ExtrusionLine& poly : polys_per_index[idx])
            {
                file << type << " " << poly.junctions.size() << " " << idx << '\n';
                for (const ExtrusionJunction& junction : poly.junctions)
                {
                    file << INT2MM(junction.p.X) << ' ' << INT2MM(junction.p.Y) << ' ' << INT2MM(junction.w) * 0.5 << " 0.0 0.0\n";
                }
            }
        }
        
        type = "open";
    }    
}

} // namespace arachne
