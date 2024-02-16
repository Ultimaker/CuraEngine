// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ANGLE_H
#define ANGLE_H

#include <cmath> //For fmod.

#include "../../utils/math.h" //For PI.

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
    AngleDegrees()
        : value_(0.0)
    {
    }

    /*
     * \brief Converts radians to degrees.
     */
    AngleDegrees(const AngleRadians& value);

    /*
     * \brief Casts a double to an AngleDegrees instance.
     */
    AngleDegrees(double value)
        : value_(std::fmod(std::fmod(value, 360) + 360, 360))
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
    double value_ = 0;
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
    AngleRadians()
        : value_(0.0)
    {
    }

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
    double value_ = 0;
};

inline AngleDegrees::AngleDegrees(const AngleRadians& value)
    : value_(value * 360 / TAU)
{
}
inline AngleRadians::AngleRadians(const AngleDegrees& value)
    : value_(double(value) * TAU / 360.0)
{
}

} // namespace cura

#endif // ANGLE_H
