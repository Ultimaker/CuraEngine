//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ANGLERADIANS_H
#define ANGLERADIANS_H

#include <cmath> //For fmod.
#include "../../utils/math.h"

#define TAU (2.0 * M_PI)

namespace cura
{

/*
 * \brief Represents an angle in radians.
 *
 * This is a facade. It behaves like a double, but this is using clock
 * arithmetic which guarantees that the value is always between 0 and 2 * pi.
 */
struct AngleRadians
{
    /*
     * \brief Default constructor setting the angle to 0.
     */
    AngleRadians() : value(0.0) {};

    /*
     * \brief Translate the double value in degrees to an AngleRadians instance.
     */
    AngleRadians(double value) : value(std::fmod(std::fmod(value, TAU) + TAU, TAU)) {};

    /*
     * \brief Casts the AngleRadians instance to a double.
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
     * \brief The actual angle, as a double.
     *
     * This value should always be between 0 and 2 * pi.
     */
    double value = 0;
};

}

#endif //ANGLERADIANS_H