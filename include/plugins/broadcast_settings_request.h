// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include "Cura.pb.h"
#include "cura/plugins/slots/broadcast/v0/broadcast.pb.h"
#include "plugins/converter.h"

namespace curaengine::plugins
{
struct broadcast_settings_request : public details::converter<broadcast_settings_request, ::cura::plugins::slots::broadcast::v0::BroadcastServiceSettingsRequest, ::cura::proto::Slice>
{
    value_type operator()(const native_value_type& slice_message) const;
};
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
