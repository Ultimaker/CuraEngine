// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <string>

#include "cura/plugins/slots/infill/v0/generate.pb.h"
#include "geometry/Shape.h"
#include "plugins/converter.h"
#include "settings/Settings.h"

namespace cura::plugins
{
struct infill_generate_request : public details::converter<infill_generate_request, slots::infill::v0::generate::CallRequest, Shape>
{
    value_type operator()(const native_value_type& inner_contour, const std::string& pattern, const Settings& settings, const coord_t z) const;
};
} // namespace cura::plugins
#endif // ENABLE_PLUGINS
