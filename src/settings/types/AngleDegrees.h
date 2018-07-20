//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ANGLEDEGREES_H
#define ANGLEDEGREES_H

#include <cmath> //For fmod.

namespace cura
{

/*
 * Represents an angle in degrees.
 *
 * This is a facade. It behaves like a double, but this is using clock
 * arithmetic which guarantees that the value is always between 0 and 360.
 */
struct AngleDegrees
{
    /*
     * Casts a double to an AngleDegrees instance.
     */
    AngleDegrees(double value) : value(std::fmod(std::fmod(value, 360) + 360, 360)) {};

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
    AngleDegrees operator +(const AngleDegrees& other) const
    {
        return std::fmod(std::fmod(value + other.value, 360) + 360, 360);
    }
    AngleDegrees& operator +=(const AngleDegrees& other)
    {
        value = std::fmod(std::fmod(value + other.value, 360) + 360, 360);
        return *this;
    }
    AngleDegrees operator -(const AngleDegrees& other) const
    {
        return std::fmod(std::fmod(value - other.value, 360) + 360, 360);
    }
    AngleDegrees& operator -=(const AngleDegrees& other)
    {
        value = std::fmod(std::fmod(value - other.value, 360) + 360, 360);
        return *this;
    }

    /*
     * The actual angle, as a double.
     *
     * This value should always be between 0 and 360.
     */
    double value = 0;
};

}

#endif //ANGLEDEGREES_H