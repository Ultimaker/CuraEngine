// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include "cura/plugins/slots/simplify/v0/modify.pb.h"
#include "geometry/Shape.h"
#include "plugins/details/converter.h"

namespace curaengine::plugins
{
struct simplify_request : public details::converter<simplify_request, ::cura::plugins::slots::simplify::v0::modify::CallRequest, ::curaengine::geometry::Shape>
{
    value_type operator()(const native_value_type& polygons, const ::curaengine::utils::coord_t max_resolution, const ::curaengine::utils::coord_t max_deviation, const ::curaengine::utils::coord_t max_area_deviation) const;
};
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
