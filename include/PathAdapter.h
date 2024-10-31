// Copyright (c) 2024 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATH_ADAPTER_H
#define PATH_ADAPTER_H

#include <stddef.h>

#include "geometry/Point2LL.h"
#include "utils/Coord_t.h"

namespace cura
{

/* Adapter class to allow extrusion-related functions to work with either an ExtrusionLine or a Polygon */
template<typename PathType>
class PathAdapter
{
public:
    /*!
     * \brief Base constructor
     * \param path The actual stored path
     * \param fixed_line_width The fixed line width in case the stored path doesn't handle information about a variable
     *                         line width. Can be omitted otherwise.
     */
    PathAdapter(const PathType& path, coord_t fixed_line_width = 0)
        : path_(path)
        , fixed_line_width_(fixed_line_width)
    {
    }

    bool empty() const
    {
        return path_.empty();
    }

    size_t size() const
    {
        return path_.size();
    }

    coord_t length() const
    {
        return path_.length();
    }

    const Point2LL& pointAt(size_t index) const;

    coord_t lineWidthAt(size_t index) const;

    const PathType& getPath() const
    {
        return path_;
    }

private:
    const PathType& path_;
    const coord_t fixed_line_width_;
};

} // namespace cura

#endif // PATH_ADAPTER_H
