// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_COMPONENTCOMMON_H
#define PLUGINS_COMPONENTCOMMON_H

#include "cura/plugins/v0/slot_id.pb.h"
#include "plugins/metadata.h"
#include "utils/format/thread_id.h"

#include <agrpc/asio_grpc.hpp>
#include <agrpc/grpc_context.hpp>

namespace cura::plugins
{

namespace details
{
using slot_info_ptr = std::shared_ptr<slot_metadata>;
using plugin_info_ptr = std::shared_ptr<std::optional<plugin_metadata>>;
} // namespace details

/**
 * @brief Prepares client_context for the remote call.
 *
 * Sets timeout for the call and adds metadata to context.
 *
 * @param client_context - Client context to prepare
 * @param timeout - Call timeout duration (optional, default = 500ms)
 */
inline static void
    prep_client_context(grpc::ClientContext& client_context, const slot_metadata& slot_info, const std::chrono::milliseconds& timeout = std::chrono::milliseconds(500))
{
    // Set time-out
    client_context.set_deadline(std::chrono::system_clock::now() + timeout);

    // Metadata
    client_context.AddMetadata("cura-engine-uuid", slot_info.engine_uuid.data());
    client_context.AddMetadata("cura-thread-id", fmt::format("{}", std::this_thread::get_id()));
}

} // namespace cura::plugins

#endif // PLUGINS_COMPONENTCOMMON_H
