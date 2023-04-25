// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_INCLUDE_PLUGINS_SLOTS_H
#define CURAENGINE_INCLUDE_PLUGINS_SLOTS_H

#include <memory>
#include <unordered_map>

#include "plugins/types.h"

namespace cura::plugins
{

class Slots
{
    constexpr Slots() noexcept = default;
    std::unordered_map<Slot, std::shared_ptr<plugin_t>> slots_{};

public:
    Slots(const Slots&) = delete;
    Slots(Slots&&) = delete;

    static Slots& instance() noexcept
    {
        static Slots instance{};
        return instance;
    }

    /*! \brief Register a plugin to a slot
     *
     * \param slot The slot to register the plugin to
     * \param plugin The plugin to register
     */
    void registerSlot(Slot slot, std::shared_ptr<plugin_t> plugin) noexcept
    {
        slots_.insert_or_assign(slot, std::move(plugin));
    }
};


} // namespace cura::plugins


#endif // CURAENGINE_INCLUDE_PLUGINS_SLOTS_H
