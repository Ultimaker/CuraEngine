// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_TYPES_ARITHMITIC_FACADE_H
#define UTILS_TYPES_ARITHMITIC_FACADE_H

#include "utils/types/generic.h"

namespace cura::utils
{

template<floating_point T>
struct NumericFacade
{
    using value_type = T;

    value_type value{};

    constexpr NumericFacade() noexcept = default;

    constexpr NumericFacade(const NumericFacade& other) noexcept = default;
    constexpr NumericFacade(NumericFacade&& other) noexcept = default;

    constexpr NumericFacade(const floating_point auto val) noexcept
        : value{ static_cast<value_type>(val) } {};
    constexpr explicit NumericFacade(const integral auto val) noexcept
        : value{ static_cast<value_type>(val) } {};

    constexpr NumericFacade& operator=(const NumericFacade& other) noexcept = default;

    constexpr NumericFacade& operator=(const floating_point auto& other) noexcept
    {
        this->value = static_cast<value_type>(other);
        return *this;
    }

    constexpr NumericFacade& operator=(NumericFacade&& other) noexcept = default;

    constexpr NumericFacade& operator=(const floating_point auto&& other) noexcept
    {
        this->value = static_cast<value_type>(other);
        return *this;
    }

    ~NumericFacade() noexcept = default;

    constexpr operator value_type() const noexcept
    {
        return value;
    }

    constexpr bool operator==(const NumericFacade& other) const noexcept
    {
        return value == other.value;
    }

    constexpr bool operator==(const floating_point auto& other) const noexcept
    {
        return value == static_cast<value_type>(other);
    }

    constexpr auto operator<=>(const NumericFacade& other) const noexcept = default;
    constexpr auto operator<=>(const floating_point auto& other) const noexcept
    {
        return value <=> static_cast<value_type>(other);
    };

    constexpr NumericFacade& operator+=(const NumericFacade& other) noexcept
    {
        value += other.value;
        return *this;
    }

    constexpr NumericFacade& operator+=(const floating_point auto& other) noexcept
    {
        value += static_cast<value_type>(other);
        return *this;
    }

    constexpr NumericFacade& operator-=(const NumericFacade& other) noexcept
    {
        value -= other.value;
        return *this;
    }

    constexpr NumericFacade& operator-=(const floating_point auto& other) noexcept
    {
        value -= static_cast<value_type>(other);
        return *this;
    }

    constexpr NumericFacade& operator*=(const NumericFacade& other) noexcept
    {
        value *= other.value;
        return *this;
    }

    constexpr NumericFacade& operator*=(const floating_point auto& other) noexcept
    {
        value *= static_cast<value_type>(other);
        return *this;
    }

    constexpr NumericFacade& operator/=(const NumericFacade& other)
    {
        value /= other.value;
        return *this;
    }

    constexpr NumericFacade& operator/=(const floating_point auto& other)
    {
        value /= static_cast<value_type>(other);
        return *this;
    }

    constexpr NumericFacade operator+(const NumericFacade& other) const noexcept
    {
        return { value + other.value };
    }

    constexpr NumericFacade operator+(NumericFacade&& other) const noexcept
    {
        return { value + other.value };
    }

    constexpr NumericFacade operator+(const floating_point auto& other) const noexcept
    {
        return { value + static_cast<value_type>(other) };
    }

    constexpr NumericFacade operator-(const NumericFacade& other) const noexcept
    {
        return { value - other.value };
    }

    constexpr NumericFacade operator-(const floating_point auto& other) const noexcept
    {
        return { value - static_cast<value_type>(other) };
    }

    constexpr NumericFacade operator*(const NumericFacade& other) const noexcept
    {
        return { value * other.value };
    }

    constexpr NumericFacade operator*(const floating_point auto& other) const noexcept
    {
        return { value * static_cast<value_type>(other) };
    }

    constexpr NumericFacade operator/(const NumericFacade& other) const
    {
        return { value / other.value };
    }

    constexpr NumericFacade operator/(const floating_point auto& other) const
    {
        return { value / static_cast<value_type>(other) };
    }

    constexpr NumericFacade operator-() const noexcept
    {
        return { -value };
    }
};

} // namespace cura::utils

#endif // UTILS_TYPES_ARITHMITIC_FACADE_H
