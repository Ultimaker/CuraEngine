// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ANGLE_H
#define ANGLE_H

#include <cmath> //For fmod.
#include <numbers>

#define TAU (2.0 * std::numbers::pi)

namespace cura
{
// AngleDegrees and AngleRadians classes defined together here interweaved, to resolve their interdependencies.
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
    constexpr AngleRadians() noexcept = default;

    /*!
     * \brief Converts an angle from degrees into radians.
     */
    constexpr AngleRadians(const AngleDegrees& value);

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

inline AngleDegrees::AngleDegrees(const AngleRadians& value)
    : value_(value.value_ * 360 / TAU)
{
}

constexpr inline AngleRadians::AngleRadians(const AngleDegrees& value)
    : value_(value.value_ * TAU / 360.0)
{
}

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

#endif // ANGLE_H
