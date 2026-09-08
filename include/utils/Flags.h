// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_FLAGS_H
#define UTILS_FLAGS_H

#include <type_traits>

/*!
 * Utility class to handle enumerations that are actually combinable bit flags. It adds convenient methods to combine the values, compare and test
 * them in a type-safe way.
 * @tparam EnumClass The enumeration class which values are to be used. All the values should set a single and different bit. There should also be
 *                   some `none` value that contains 0.
 */
template<typename EnumClass>
class Flags
{
public:
    using enum_int_type = std::underlying_type_t<EnumClass>;

public:
    Flags() = default;

    constexpr Flags(const EnumClass value)
        : value_(static_cast<enum_int_type>(value))
    {
    }

    Flags(const Flags& other) = default;

    enum_int_type value() const
    {
        return value_;
    }

    Flags operator|(const Flags& other) const
    {
        return Flags(static_cast<EnumClass>(value_ | other.value_));
    }

    void operator|=(const Flags other)
    {
        value_ |= other.value_;
    }

    bool operator&(const EnumClass& value) const
    {
        return (value_ & static_cast<enum_int_type>(value)) != 0;
    }

    bool operator==(const Flags& other) const
    {
        return value_ == other.value_;
    }

private:
    enum_int_type value_{ 0 };
};

#endif