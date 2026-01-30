// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/OpenLinesSet.h"


namespace cura
{

OpenLinesSet::OpenLinesSet(ClipperLib::Paths&& paths)
{
    reserve(paths.size());
    for (ClipperLib::Path& path : paths)
    {
        emplace_back(std::move(path));
    }
}

void OpenLinesSet::addSegment(const Point2LL& from, const Point2LL& to)
{
    emplace_back(std::initializer_list<Point2LL>{ from, to });
}

void OpenLinesSet::split(const size_t line_index, const size_t point_index)
{
    if (line_index >= size())
    {
        return;
    }

    OpenPolyline& line = at(line_index);
    if (! line.isValid() || point_index == 0 || point_index >= line.size() - 1) // Can't split at start or end
    {
        return;
    }

    OpenPolyline tail(ClipperLib::Path(std::make_move_iterator(line.begin() + point_index), std::make_move_iterator(line.end())));
    line.resize(point_index + 1);
    line[point_index] = tail.front(); // The move may have left the point in an indeterminate state, so we need to assign it back
    insert(begin() + line_index + 1, std::move(tail));
}

} // namespace cura
