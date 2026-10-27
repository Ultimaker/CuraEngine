// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <tuple>
#include <vector>

#include "WallToolPaths.h"
#include "cura/plugins/slots/infill/v0/generate.pb.h"
#include "geometry/OpenLinesSet.h"
#include "geometry/Shape.h"
#include "plugins/converter.h"

namespace cura::plugins
{
struct infill_generate_response
    : public details::converter<infill_generate_response, slots::infill::v0::generate::CallResponse, std::tuple<std::vector<std::vector<ExtrusionLine>>, Shape, OpenLinesSet>>
{
    native_value_type operator()(const value_type& message) const;
};
} // namespace cura::plugins
#endif // ENABLE_PLUGINS
