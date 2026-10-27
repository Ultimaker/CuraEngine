// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <string>

#include "cura/plugins/slots/postprocess/v0/modify.pb.h"
#include "plugins/converter.h"

namespace cura::plugins
{
struct postprocess_response : public details::converter<postprocess_response, slots::postprocess::v0::modify::CallResponse, std::string>
{
    native_value_type operator()([[maybe_unused]] const native_value_type& original_value, const value_type& message) const;
};
} // namespace cura::plugins
#endif // ENABLE_PLUGINS
