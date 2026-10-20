// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <algorithm>
#include <cmath> //For fmod.
#include <numbers>

#define TAU (2.0 * std::numbers::pi)

namespace cura
{
class AngleDegrees;

/*
 * \brief Represents an angle in radians.
 *
 * This is a facade. It behaves like a double, but this is using clock
 * arithmetic which guarantees that the value is always between 0 and 2 * pi.
 */
class AngleRadians
{
public:
    /*
     * \brief Default constructor setting the angle to 0.
     */
    AngleRadians() noexcept = default;

    /*!
     * \brief Converts an angle from degrees into radians.
     */
    AngleRadians(const AngleDegrees& value);

    /*
     * \brief Translate the double value in degrees to an AngleRadians instance.
     */
    AngleRadians(double value)
        : value_(std::fmod(std::fmod(value, TAU) + TAU, TAU))
    {
    }

    /*
     * \brief Casts the AngleRadians instance to a double.
     */
    operator double() const
    {
        return value_;
    }

    /*
     * Some operators implementing the clock arithmetic.
     */
    AngleRadians operator+(const AngleRadians& other) const
    {
        return std::fmod(std::fmod(value_ + other.value_, TAU) + TAU, TAU);
    }

    AngleRadians& operator+=(const AngleRadians& other)
    {
        value_ = std::fmod(std::fmod(value_ + other.value_, TAU) + TAU, TAU);
        return *this;
    }

    AngleRadians operator-(const AngleRadians& other) const
    {
        return std::fmod(std::fmod(value_ - other.value_, TAU) + TAU, TAU);
    }

    AngleRadians& operator-=(const AngleRadians& other)
    {
        value_ = std::fmod(std::fmod(value_ - other.value_, TAU) + TAU, TAU);
        return *this;
    }

    /*
     * \brief The actual angle, as a double.
     *
     * This value should always be between 0 and 2 * pi.
     */
    double value_{ 0 };
};

/*!
 * \brief Safe call to "std::tan" which limits the higher angle value to something slightly less that π/2 so that when
 *        the given angle is higher that this value, the returned value is not a huge number
 * \param angle The input angle, which should be [0, π/2]
 * \return The tangent value of the angle, limited
 * \note This method exists as a convenience because this is a common case in the engine, as we have many settings that
 *       are angles setup on [0, π/2] and which translate to a distance
 */
inline double boundedTan(const AngleRadians& angle)
{
    return std::tan(std::min(static_cast<double>(angle), std::numbers::pi / 2.0 - 0.001));
}

} // namespace cura

// Include after the class definition so both conversion constructors work in either include order.
#include "settings/types/AngleDegrees.h"

namespace cura
{

inline AngleRadians::AngleRadians(const AngleDegrees& value)
    : value_(value.value_ * TAU / 360.0)
{
}

} // namespace cura
