//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef ANGLEDEGREES_H
#define ANGLEDEGREES_H

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
    LayerIndex(double value) : value(((value % 360) + 360) % 360) {};

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
    operator +(const AngleDegrees& other) const
    {
        return (((value + other.value) % 360) + 360) % 360;
    }
    operator +=(const AngleDegrees& other)
    {
        value = (((value + other.value) % 360) + 360) % 360;
    }
    operator -(const AngleDegrees& other) const
    {
        return (((value - other.value) % 360) + 360) % 360;
    }
    operator -=(const AngleDegrees& other)
    {
        value = (((value - other.value) % 360) + 360) % 360;
    }

    /*
     * The actual angle, as a double.
     *
     * This value should always be between 0 and 360.
     */
    double value = 0;
};

#endif //ANGLEDEGREES_H