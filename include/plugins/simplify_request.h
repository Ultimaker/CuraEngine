// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include "cura/plugins/slots/simplify/v0/modify.pb.h"
#include "geometry/Shape.h"
#include "plugins/converter.h"

namespace cura::plugins
{
struct simplify_request : public details::converter<simplify_request, slots::simplify::v0::modify::CallRequest, Shape>
{
    value_type operator()(const native_value_type& polygons, const coord_t max_resolution, const coord_t max_deviation, const coord_t max_area_deviation) const;
};
} // namespace cura::plugins
#endif // ENABLE_PLUGINS
