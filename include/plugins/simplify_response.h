// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include "cura/plugins/slots/simplify/v0/modify.pb.h"
#include "geometry/Shape.h"
#include "plugins/details/converter.h"

namespace curaengine::plugins
{
struct simplify_response : public details::converter<simplify_response, ::cura::plugins::slots::simplify::v0::modify::CallResponse, geometry::Shape>
{
    native_value_type operator()([[maybe_unused]] const native_value_type& original_value, const value_type& message) const;
};
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
