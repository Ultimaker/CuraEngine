// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <bit>
#include <optional>

#include "ExtruderNumber.h"
#include "settings/Settings.h"

namespace cura
{

class ExtrudersSet
{
public:
    class Iterator
    {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = ExtruderNumber;
        using difference_type = ptrdiff_t;
        using pointer = const ExtruderNumber*;
        using reference = const ExtruderNumber&;

        static Iterator begin(const EXTRUDERS_BITMASK_TYPE extruders_mask)
        {
            return Iterator(extruders_mask, findSetPosition(extruders_mask));
        }

        static Iterator end(const EXTRUDERS_BITMASK_TYPE extruders_mask)
        {
            return Iterator(extruders_mask, std::nullopt);
        }

        reference operator*() const
        {
            return position_.value();
        }

        Iterator& operator++()
        {
            if (! position_.has_value())
            {
                return *this;
            }

            // Clear all the LSBs starting from current position
            const EXTRUDERS_BITMASK_TYPE next_position_mask = (extruders_mask_ >> (position_.value() + 1)) << (position_.value() + 1);
            position_ = findSetPosition(next_position_mask);
            return *this;
        }

        bool operator!=(const Iterator& other) const = default;

        bool operator==(const Iterator& other) const = default;

    private:
        Iterator(const EXTRUDERS_BITMASK_TYPE extruders_mask, const std::optional<ExtruderNumber> position)
            : extruders_mask_(extruders_mask)
            , position_(position)
        {
        }

        static std::optional<ExtruderNumber> findSetPosition(const EXTRUDERS_BITMASK_TYPE extruders_mask)
        {
            return extruders_mask ? std::make_optional(std::countr_zero(extruders_mask)) : std::nullopt;
        }

    private:
        EXTRUDERS_BITMASK_TYPE extruders_mask_;
        std::optional<ExtruderNumber> position_;
    };

public:
    using iterator = Iterator;
    using const_iterator = Iterator;
    using value_type = Iterator::value_type;
    using difference_type = Iterator::difference_type;
    using size_type = size_t;

public:
    bool contains(const ExtruderNumber extruder_nr) const
    {
        return extruders_mask_ & (static_cast<EXTRUDERS_BITMASK_TYPE>(1) << extruder_nr);
    }

    void set(const ExtruderNumber extruder_nr)
    {
        extruders_mask_ |= (static_cast<EXTRUDERS_BITMASK_TYPE>(1) << extruder_nr);
    }

    void unset(const ExtruderNumber extruder_nr)
    {
        extruders_mask_ &= ~(static_cast<EXTRUDERS_BITMASK_TYPE>(1) << extruder_nr);
    }

    bool empty() const
    {
        return extruders_mask_ == 0;
    }

    iterator begin()
    {
        return iterator::begin(extruders_mask_);
    }

    const_iterator begin() const
    {
        return const_iterator::begin(extruders_mask_);
    }

    iterator end()
    {
        return iterator::end(extruders_mask_);
    }

    const_iterator end() const
    {
        return const_iterator::end(extruders_mask_);
    }

private:
    EXTRUDERS_BITMASK_TYPE extruders_mask_{};
};

} // namespace cura
