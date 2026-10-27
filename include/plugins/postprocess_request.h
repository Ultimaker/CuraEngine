// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <string>

#include "cura/plugins/slots/postprocess/v0/modify.pb.h"
#include "plugins/converter.h"

namespace cura::plugins
{
struct postprocess_request : public details::converter<postprocess_request, slots::postprocess::v0::modify::CallRequest, std::string>
{
    value_type operator()(const native_value_type& gcode) const;
};
} // namespace cura::plugins
#endif // ENABLE_PLUGINS
