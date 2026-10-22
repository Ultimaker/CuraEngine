// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include "cura/plugins/slots/simplify/v0/modify.pb.h"
#include "geometry/Shape.h"
#include "plugins/converter.h"

namespace cura::plugins
{
struct simplify_response : public details::converter<simplify_response, slots::simplify::v0::modify::CallResponse, Shape>
{
    native_value_type operator()([[maybe_unused]] const native_value_type& original_value, const value_type& message) const;
};
} // namespace cura::plugins
#endif // ENABLE_PLUGINS
