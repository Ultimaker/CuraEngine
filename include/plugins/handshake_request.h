// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <string>

#include "cura/plugins/slots/handshake/v0/handshake.pb.h"
#include "plugins/converter.h"
#include "plugins/slot_metadata.h"

namespace curaengine::plugins
{
struct handshake_request : public details::converter<handshake_request, ::cura::plugins::slots::handshake::v0::CallRequest, slot_metadata>
{
    value_type operator()(const std::string& name, const std::string& version, const native_value_type& slot_info) const;
};
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
