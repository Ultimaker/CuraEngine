//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef VELOCITY_H
#define VELOCITY_H

#include <algorithm> //For std::max.

namespace cura
{

/*
 * Represents a velocity in millimetres per second.
 *
 * This is a facade. It behaves like a double, only it can't be negative.
 */
struct Velocity
{
    /*
     * Casts a double to a Velocity instance.
     */
    Velocity(double value) : value(std::max(value, 0.0)) {};

    /*
     * Casts the Temperature instance to a double.
     */
    operator double() const
    {
        return value;
    }

    /*
     * The actual temperature, as a double.
     */
    double value = 0;
};

}

#endif //VELOCITY_H