/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef SLICER_SLICER_SEGMENT_H
#define SLICER_SLICER_SEGMENT_H

#include <functional>

#include "../utils/intpoint.h"

namespace cura
{

class SlicerSegment
{
public:
    Point start, end;
    int faceIndex = -1;
    // The index of the other face connected via the edge that created end
    int endOtherFaceIdx = -1;
    // If end corresponds to a vertex of the mesh, then this is populated
    // with the vertex that it ended on.
    const MeshVertex *endVertex = nullptr;
    bool addedToPolygon = false;
};

} // namespace cura

namespace std
{
    template<> struct hash<cura::SlicerSegment>
    {
        typedef std::size_t result_type;
        result_type operator()(cura::SlicerSegment const& s) const
        {
//             result_type const h1 ( std::hash<std::string>()(s.first_name) );
//             result_type const h2 ( std::hash<std::string>()(s.last_name) );
//             return h1 ^ (h2 << 1); // or use boost::hash_combine
            return std::hash<cura::Point>()(cura::operator+(s.start, s.end));
        }
    };
} // namespace std


#endif // SLICER_SLICER_SEGMENT_H