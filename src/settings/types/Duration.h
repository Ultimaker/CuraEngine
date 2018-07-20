//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef DURATION_H
#define DURATION_H

namespace cura
{

/*
 * Represents a duration in seconds.
 *
 * This is a facade. It behaves like a double, only it can't be negative.
 */
struct Duration
{
    /*
     * Casts a double to a Duration instance.
     */
    Duration(double value) : value(std::max(value, 0.0)) {};

    /*
     * Casts the Duration instance to a double.
     */
    operator double() const
    {
        return value;
    }

    /*
     * The actual duration, as a double.
     */
    double value = 0;
};

}

#endif //DURATION_H