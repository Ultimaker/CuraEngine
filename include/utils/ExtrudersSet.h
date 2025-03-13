// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <bit>
#include <optional>

#include "settings/Settings.h"
#include "ExtruderNumber.h"

namespace cura
{

class ExtrudersSet
{
public:
    class Iterator
    {
    public:
        Iterator(const EXTRUDERS_BITMASK_TYPE extruders_mask, const std::optional<ExtruderNumber> position = std::nullopt) :
            extruders_mask_(extruders_mask),
            position_(position.value_or(findSetPosition(extruders_mask_)))
        {
        }

        ExtruderNumber operator*() const
        {
            return position_;
        }

        Iterator& operator++()
        {
            // Clear all the LSBs starting from current position
            const EXTRUDERS_BITMASK_TYPE next_position_mask = (extruders_mask_ >> (position_ + 1)) << (position_ + 1);
            position_ = findSetPosition(next_position_mask);
            return *this;
        }

        bool operator!=(const Iterator& other) const
        {
            return position_ != other.position_;
        }

    public:
        static constexpr ExtruderNumber end_position = sizeof(EXTRUDERS_BITMASK_TYPE) * 8;

    private:
        static ExtruderNumber findSetPosition(const EXTRUDERS_BITMASK_TYPE extruders_mask)
        {
            return extruders_mask ? std::countr_zero(extruders_mask) : end_position;
        }

    private:
        EXTRUDERS_BITMASK_TYPE extruders_mask_;
        ExtruderNumber position_;
    };

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

    Iterator begin() const
    {
        return Iterator(extruders_mask_);
    }

    Iterator end() const
    {
        return Iterator(extruders_mask_, Iterator::end_position);
    }

private:
    EXTRUDERS_BITMASK_TYPE extruders_mask_{};
};

} // namespace cura
