// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef LAYERINDEX_H
#define LAYERINDEX_H

#include "utils/types/generic.h"

#include <functional>

namespace cura
{

struct LayerIndex
{
    using value_type = int64_t;
    using difference_type = std::ptrdiff_t;

    value_type value{};

    constexpr LayerIndex() noexcept = default;

    constexpr LayerIndex(const LayerIndex& other) noexcept = default;
    constexpr LayerIndex(LayerIndex&& other) noexcept = default;

    constexpr explicit LayerIndex(const utils::floating_point auto val) noexcept
        : value{ static_cast<value_type>(val) } {};

    constexpr LayerIndex(const utils::integral auto val) noexcept
        : value{ static_cast<value_type>(val) } {};

    constexpr LayerIndex& operator=(const LayerIndex& other) noexcept = default;

    constexpr LayerIndex& operator=(const utils::integral auto& other) noexcept
    {
        this->value = static_cast<value_type>(other);
        return *this;
    }

    constexpr LayerIndex& operator=(LayerIndex&& other) noexcept = default;
    constexpr LayerIndex& operator=(const utils::integral auto&& other) noexcept
    {
        this->value = static_cast<value_type>(other);
        return *this;
    }

    ~LayerIndex() noexcept = default;

    constexpr operator value_type() const noexcept
    {
        return value;
    }

    constexpr bool operator==(const LayerIndex& other) const noexcept
    {
        return value == other.value;
    }

    constexpr bool operator==(const utils::integral auto& other) const noexcept
    {
        return value == static_cast<value_type>(other);
    }

    constexpr auto operator<=>(const LayerIndex& other) const noexcept = default;
    constexpr auto operator<=>(const utils::integral auto& other) const noexcept
    {
        return value <=> static_cast<value_type>(other);
    };

    constexpr LayerIndex& operator+=(const LayerIndex& other) noexcept
    {
        value += other.value;
        return *this;
    }

    constexpr LayerIndex& operator+=(const utils::integral auto& other) noexcept
    {
        value += static_cast<value_type>(other);
        return *this;
    }

    constexpr LayerIndex& operator-=(const LayerIndex& other) noexcept
    {
        value -= other.value;
        return *this;
    }

    constexpr LayerIndex& operator-=(const utils::integral auto& other) noexcept
    {
        value -= static_cast<value_type>(other);
        return *this;
    }

    constexpr LayerIndex& operator*=(const LayerIndex& other) noexcept
    {
        value *= other.value;
        return *this;
    }

    constexpr LayerIndex& operator*=(const utils::integral auto& other) noexcept
    {
        value *= static_cast<value_type>(other);
        return *this;
    }

    constexpr LayerIndex& operator/=(const LayerIndex& other)
    {
        value /= other.value;
        return *this;
    }

    constexpr LayerIndex& operator/=(const utils::integral auto& other)
    {
        value /= static_cast<value_type>(other);
        return *this;
    }

    constexpr LayerIndex operator+(const LayerIndex& other) const noexcept
    {
        return { value + other.value };
    }

    constexpr LayerIndex operator+(LayerIndex&& other) const noexcept
    {
        return { value + other.value };
    }

    constexpr LayerIndex operator+(const utils::integral auto& other) const noexcept
    {
        return { value + static_cast<value_type>(other) };
    }

    constexpr LayerIndex operator-(const LayerIndex& other) const noexcept
    {
        return { value - other.value };
    }

    constexpr LayerIndex operator-(const utils::integral auto& other) const noexcept
    {
        return { value - static_cast<value_type>(other) };
    }

    constexpr LayerIndex operator*(const LayerIndex& other) const noexcept
    {
        return { value * other.value };
    }

    constexpr LayerIndex operator*(const utils::integral auto& other) const noexcept
    {
        return { value * static_cast<value_type>(other) };
    }

    constexpr LayerIndex operator/(const LayerIndex& other) const
    {
        return { value / other.value };
    }

    constexpr LayerIndex operator/(const utils::integral auto& other) const
    {
        return { value / static_cast<value_type>(other) };
    }

    constexpr LayerIndex operator-() const noexcept
    {
        return { -value };
    }

    constexpr LayerIndex& operator++() noexcept
    {
        ++value;
        return *this;
    }

    LayerIndex operator++(int) noexcept
    {
        LayerIndex tmp{ *this };
        operator++();
        return tmp;
    }

    constexpr LayerIndex& operator--() noexcept
    {
        --value;
        return *this;
    }

    LayerIndex operator--(int) noexcept
    {
        LayerIndex tmp{ *this };
        operator--();
        return tmp;
    }
};

} // namespace cura

namespace std
{
template<>
struct hash<cura::LayerIndex>
{
    auto operator()(const cura::LayerIndex& layer_index) const
    {
        return hash<decltype(layer_index.value)>()(layer_index.value);
    }
};
} // namespace std

#endif // LAYERINDEX_H
