// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <tuple>
#include <vector>

#include "WallToolPaths.h"
#include "cura/plugins/slots/infill/v0/generate.pb.h"
#include "plugins/details/converter.h"
#include "types/geometry/OpenLinesSet.h"
#include "types/geometry/Shape.h"

namespace curaengine::plugins
{
struct infill_generate_response : public details::converter<
                                      infill_generate_response,
                                      ::cura::plugins::slots::infill::v0::generate::CallResponse,
                                      std::tuple<std::vector<std::vector<types::geometry::ExtrusionLine>>, types::geometry::Shape, types::geometry::OpenLinesSet>>
{
    native_value_type operator()(const value_type& message) const;
};
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
