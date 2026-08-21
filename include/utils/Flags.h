// Copyright (c) 2026 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_FLAGS_H
#define UTILS_FLAGS_H

#include <type_traits>

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