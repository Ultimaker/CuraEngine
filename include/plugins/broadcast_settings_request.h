// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include "cura/plugins/slots/broadcast/v0/broadcast.pb.h"
#include "plugins/details/converter.h"

namespace curaengine
{
class Scene;
}

namespace curaengine::plugins
{
struct broadcast_settings_request
    : public details::converter<broadcast_settings_request, ::cura::plugins::slots::broadcast::v0::BroadcastServiceSettingsRequest, ::curaengine::Scene>
{
    value_type operator()(const native_value_type& scene) const;
};
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
