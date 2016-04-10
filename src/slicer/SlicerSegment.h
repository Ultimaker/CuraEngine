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
    int faceIndex;
    bool addedToPolygon;
    
    bool operator==(const SlicerSegment& b) const
    {
        return start == b.start && end == b.end;
    }
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