// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <string>

#include "Settings.h"
#include "cura/plugins/slots/infill/v0/generate.pb.h"
#include "geometry/Shape.h"
#include "plugins/details/converter.h"

namespace curaengine::plugins
{
struct infill_generate_request : public details::converter<infill_generate_request, ::cura::plugins::slots::infill::v0::generate::CallRequest, geometry::Shape>
{
    value_type operator()(const native_value_type& inner_contour, const std::string& pattern, const Settings& settings, const utils::coord_t z) const;
};
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
