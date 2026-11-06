// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <fmt/format.h>

#include "cura/plugins/v0/slot_id.pb.h"

namespace cura::plugins::v0
{

constexpr auto format_as(SlotID id)
{
    return fmt::underlying(id);
}

} // namespace cura::plugins::v0
