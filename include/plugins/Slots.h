// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#ifdef ENABLE_PLUGINS
#include <exception>
#include <memory>
#include <tuple>
#include <utility>

#include "cura/plugins/slots/broadcast/v0/broadcast.grpc.pb.h"
#include "cura/plugins/slots/gcode_paths/v0/modify.grpc.pb.h"
#include "cura/plugins/slots/infill/v0/generate.grpc.pb.h"
#include "cura/plugins/slots/postprocess/v0/modify.grpc.pb.h"
#include "cura/plugins/slots/simplify/v0/modify.grpc.pb.h"
#include "cura/plugins/v0/slot_id.pb.h"
#include "plugins/v0/SlotID.h"
#include "plugins/details/Holder.h"
#include "plugins/details/SingletonRegistry.h"
#include "plugins/SlotProxy.h"
#include "plugins/details/Typelist.h"
#include "plugins/Validator.h"
#include "plugins/broadcast_settings_request.h"
#include "plugins/details/default_process.h"
#include "plugins/empty.h"
#include "plugins/gcode_paths_modify_request.h"
#include "plugins/gcode_paths_modify_response.h"
#include "plugins/details/infill_generate_default.h"
#include "plugins/infill_generate_request.h"
#include "plugins/infill_generate_response.h"
#include "plugins/postprocess_request.h"
#include "plugins/postprocess_response.h"
#include "plugins/details/simplify_default.h"
#include "plugins/simplify_request.h"
#include "plugins/simplify_response.h"
#include "plugins/types.h"

namespace curaengine
{
namespace plugins
{
namespace details
{
/**
 * @brief Alias for the Simplify slot.
 *
 * This alias represents the Simplify slot, which is used for simplifying polygons.
 *
 * @tparam Default The default behavior when no plugin is registered.
 */
template<class Default = default_process>
using slot_simplify_
    = SlotProxy<v0::SlotID::SIMPLIFY_MODIFY, "0.1.0-alpha", ::cura::plugins::slots::simplify::v0::modify::SimplifyModifyService::Stub, Validator, simplify_request, simplify_response, Default>;

template<class Default = default_process>
using slot_infill_generate_ = SlotProxy<
    v0::SlotID::INFILL_GENERATE,
    "0.1.0-alpha",
    ::cura::plugins::slots::infill::v0::generate::InfillGenerateService::Stub,
    Validator,
    infill_generate_request,
    infill_generate_response,
    Default>;

/**
 * @brief Alias for the Postprocess slot.
 *
 * This alias represents the Postprocess slot, which is used for post-processing G-code.
 *
 * @tparam Default The default behavior when no plugin is registered.
 */
template<class Default = default_process>
using slot_postprocess_ = SlotProxy<
    v0::SlotID::POSTPROCESS_MODIFY,
    "0.1.0-alpha",
    ::cura::plugins::slots::postprocess::v0::modify::PostprocessModifyService::Stub,
    Validator,
    postprocess_request,
    postprocess_response,
    Default>;

template<class Default = default_process>
using slot_settings_broadcast_
    = SlotProxy<v0::SlotID::SETTINGS_BROADCAST, "0.1.0-alpha", ::cura::plugins::slots::broadcast::v0::BroadcastService::Stub, Validator, broadcast_settings_request, empty, Default>;

template<class Default = default_process>
using slot_gcode_paths_modify_ = SlotProxy<
    v0::SlotID::GCODE_PATHS_MODIFY,
    "0.1.0-alpha",
    ::cura::plugins::slots::gcode_paths::v0::modify::GCodePathsModifyService::Stub,
    Validator,
    gcode_paths_modify_request,
    gcode_paths_modify_response,
    Default>;

} // namespace curaengine::plugins::details

using slot_gcode_paths_modify = details::slot_gcode_paths_modify_<>;
using slot_infill_generate = details::slot_infill_generate_<details::infill_generate_default>;
using slot_postprocess = details::slot_postprocess_<>;
using slot_settings_broadcast = details::slot_settings_broadcast_<>;
using slot_simplify = details::slot_simplify_<details::simplify_default>;

using SlotTypes = details::Typelist<slot_gcode_paths_modify, slot_infill_generate, slot_postprocess, slot_settings_broadcast, slot_simplify>;

using slots = details::SingletonRegistry<SlotTypes, details::Holder>;

} // namespace curaengine::plugins

} // namespace curaengine

#else // No Engine plugin support
#include "plugins/details/Slots.h"

namespace curaengine::plugins::slots
{
constexpr ::curaengine::plugins::details::Slots instance() noexcept
{
    return {};
}
} // namespace curaengine::plugins::slots
#endif // ENABLE_PLUGINS
