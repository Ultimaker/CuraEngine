//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ANGLERADIANS_H
#define ANGLERADIANS_H

#define TAU 6.283185307179586477 //2 * pi

namespace cura
{

/*
 * Represents an angle in radians.
 *
 * This is a facade. It behaves like a double, but this is using clock
 * arithmetic which guarantees that the value is always between 0 and 2 * pi.
 */
struct AngleRadians
{
    /*
     * Casts a double to an AngleDegrees instance.
     */
    AngleRadians(double value) : value(std::fmod(std::fmod(value, TAU) + TAU, TAU)) {};

    /*
     * Casts the AngleDegrees instance to a double.
     */
    operator double() const
    {
        return value;
    }

    /*
     * Some operators implementing the clock arithmetic.
     */
    AngleRadians operator +(const AngleRadians& other) const
    {
        return std::fmod(std::fmod(value + other.value, TAU) + TAU, TAU);
    }
    AngleRadians& operator +=(const AngleRadians& other)
    {
        value = std::fmod(std::fmod(value + other.value, TAU) + TAU, TAU);
        return *this;
    }
    AngleRadians operator -(const AngleRadians& other) const
    {
        return std::fmod(std::fmod(value - other.value, TAU) + TAU, TAU);
    }
    AngleRadians& operator -=(const AngleRadians& other)
    {
        value = std::fmod(std::fmod(value - other.value, TAU) + TAU, TAU);
        return *this;
    }

    /*
     * The actual angle, as a double.
     *
     * This value should always be between 0 and 2 * pi.
     */
    double value = 0;
};

}

#endif //ANGLERADIANS_H