// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <string_view>

#include "cura/plugins/slots/handshake/v0/handshake.pb.h"
#include "plugins/converter.h"
#include "plugins/plugin_metadata.h"

namespace cura::plugins
{
struct handshake_response : public details::converter<handshake_response, slots::handshake::v0::CallResponse, plugin_metadata>
{
    native_value_type operator()(const value_type& message, std::string_view peer) const;
};
} // namespace cura::plugins
#endif // ENABLE_PLUGINS
