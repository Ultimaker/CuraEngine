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

    /*!
     * Iterates over the set bits, from least significant to most significant.
     * Each dereference returns a single-bit enum value.
     */
    class Iterator
    {
        using unsigned_type = std::make_unsigned_t<enum_int_type>;

    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = EnumClass;
        using difference_type = std::ptrdiff_t;
        using pointer = void;
        using reference = EnumClass;

        constexpr Iterator() = default;

        explicit constexpr Iterator(const enum_int_type value)
            : remaining_(static_cast<unsigned_type>(value))
        {
        }

        constexpr EnumClass operator*() const
        {
            const auto bit = static_cast<unsigned_type>(remaining_ & static_cast<unsigned_type>(unsigned_type{ 0 } - remaining_));
            return static_cast<EnumClass>(bit);
        }

        constexpr Iterator& operator++()
        {
            remaining_ &= static_cast<unsigned_type>(remaining_ - 1);
            return *this;
        }

        constexpr Iterator operator++(int)
        {
            const auto previous = *this;
            ++*this;
            return previous;
        }

        constexpr bool operator==(const Iterator& other) const = default;

    private:
        unsigned_type remaining_{ 0 };
    };

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

    bool isSet(const EnumClass value) const
    {
        return *this & value;
    }

    constexpr Flags operator|(const Flags& other) const
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

    constexpr Iterator begin() const
    {
        return Iterator(value_);
    }

    constexpr Iterator end() const
    {
        return Iterator();
    }

private:
    enum_int_type value_{ 0 };
};

#endif