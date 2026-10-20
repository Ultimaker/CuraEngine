// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <cmath> //For fmod.

namespace cura
{
class AngleRadians;

/*
 * \brief Represents an angle in degrees.
 *
 * This is a facade. It behaves like a double, but this is using clock
 * arithmetic which guarantees that the value is always between 0 and 360.
 */
class AngleDegrees
{
public:
    /*
     * \brief Default constructor setting the angle to 0.
     */
    AngleDegrees() noexcept = default;

    /*
     * \brief Converts radians to degrees.
     */
    AngleDegrees(const AngleRadians& value);

    /*
     * \brief Casts a double to an AngleDegrees instance.
     */
    AngleDegrees(double value)
        : value_{ std::fmod(std::fmod(value, 360) + 360, 360) }
    {
    }

    /*
     * \brief Casts the AngleDegrees instance to a double.
     */
    operator double() const
    {
        return value_;
    }

    /*
     * Some operators implementing the clock arithmetic.
     */
    AngleDegrees operator+(const AngleDegrees& other) const
    {
        return std::fmod(std::fmod(value_ + other.value_, 360) + 360, 360);
    }

    template<class T>
    AngleDegrees operator+(const T& other) const
    {
        return operator+(AngleDegrees(static_cast<double>(other)));
    }

    AngleDegrees& operator+=(const AngleDegrees& other)
    {
        value_ = std::fmod(std::fmod(value_ + other.value_, 360) + 360, 360);
        return *this;
    }
    AngleDegrees operator-(const AngleDegrees& other) const
    {
        return std::fmod(std::fmod(value_ - other.value_, 360) + 360, 360);
    }

    template<class T>
    AngleDegrees operator-(const T& other) const
    {
        return operator-(AngleDegrees(static_cast<double>(other)));
    }

    AngleDegrees& operator-=(const AngleDegrees& other)
    {
        value_ = std::fmod(std::fmod(value_ - other.value_, 360) + 360, 360);
        return *this;
    }

    /*
     * \brief The actual angle, as a double.
     *
     * This value should always be between 0 and 360.
     */
    double value_{ 0 };
};

} // namespace cura

// Include after the class definition so both conversion constructors work in either include order.
#include "settings/types/AngleRadians.h"

namespace cura
{

inline AngleDegrees::AngleDegrees(const AngleRadians& value)
    : value_(value.value_ * 360 / TAU)
{
}

} // namespace cura
