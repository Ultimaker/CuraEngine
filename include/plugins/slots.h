// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PLUGINS_SLOTS_H
#define PLUGINS_SLOTS_H

#include <memory>
#include <unordered_map>
#include <variant>

#include <range/v3/range/primitives.hpp>
#include <range/v3/utility/semiregular.hpp>

#include "plugins/converters.h"
#include "plugins/slotproxy.h"
#include "plugins/types.h"
#include "plugins/validator.h"

#include "plugin.grpc.pb.h"
#include "plugin.pb.h"

#include "utils/Simplify.h" // TODO: remove need for including implementation headers

namespace cura::plugins
{
//namespace details
//{
//constexpr auto process_default = [](auto&& args...){ return std::forward<decltype(args)>(args); };
//}

template<auto Default>
using simplify_slot = SlotProxy<SlotID::SIMPLIFY,
                                Validator<"<=0.0.1", "qwerty-azerty-temp-hash">,
                                proto::Simplify::Stub,
                                agrpc::RPC<&proto::Simplify::Stub::PrepareAsyncSimplify>,
                                simplify_request,
                                simplify_response,
                                Default>;

template<auto Default>
using postprocess_slot = SlotProxy<SlotID::POSTPROCESS,
                                   Validator<">=1.0.0 <2.0.0 || >3.2.1", "qwerty-azerty-temp-hash">,
                                   proto::Postprocess::Stub,
                                   agrpc::RPC<&proto::Postprocess::Stub::PrepareAsyncPostprocess>,
                                   postprocess_request,
                                   postprocess_response,
                                   Default>;

template<class Simplify, class Postprocess>
class Slots
{
    using slots_t = std::variant<Simplify, Postprocess>;//, Postprocess>;

    constexpr Slots() noexcept = default;
    std::unordered_map<SlotID, slots_t> slots_{};

public:
    Slots(const Slots&) = delete;
    Slots(Slots&&) = delete;

    static Slots& instance() noexcept
    {
        static Slots instance{};
        return instance;
    }

    constexpr void set(auto&& plugin)
    {
        slots_.emplace(plugin.slot_id, std::forward<decltype(plugin)>(plugin));
    }

    template<plugins::SlotID SlotID>
    constexpr auto get() const
    {
        return std::get<SlotID>(slots_.at(SlotID));
    }
};
} // namespace cura::plugins

#endif // PLUGINS_SLOTS_H
