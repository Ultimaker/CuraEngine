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

    SlicerSegment() //!< non-initializing constructor
    {}
    SlicerSegment(Point start, Point end) //!< partially initializing constructor
    : start(start)
    , end(end)
    {}
    /*!
     * equivalence testing irrespective of start/end order
     */
    bool operator==(const SlicerSegment& b) const
    {
        return (start == b.start && end == b.end) || (start == b.end && end == b.start);
    }
};

} // namespace cura

namespace std
{
    /*!
     * hash function irrespective of start/end order
     */
    template<> struct hash<cura::SlicerSegment>
    {
        typedef std::size_t result_type;
        result_type operator()(cura::SlicerSegment const& s) const
        {
            return std::hash<cura::Point>()(cura::operator+(s.start, s.end));
        }
    };
} // namespace std


#endif // SLICER_SLICER_SEGMENT_H