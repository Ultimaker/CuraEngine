// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ENABLE_PLUGINS
#include "plugins/simplify_response.h"

#include "geometry/Polygon.h"

using namespace curaengine::geometry;

namespace curaengine::plugins
{
simplify_response::native_value_type
    simplify_response::operator()([[maybe_unused]] const simplify_response::native_value_type& original_value, const simplify_response::value_type& message) const
{
    native_value_type poly{};
    for (const auto& paths : message.polygons().polygons())
    {
        Polygon o{};
        for (const auto& point : paths.outline().path())
        {
            o.push_back(Point2LL{ point.x(), point.y() });
        }
        poly.push_back(o);

        for (const auto& hole : paths.holes())
        {
            Polygon h{};
            for (const auto& point : hole.path())
            {
                h.push_back(Point2LL{ point.x(), point.y() });
            }
            poly.push_back(h);
        }
    }
    return poly;
}
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
