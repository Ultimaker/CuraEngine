//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef RATIO_H
#define RATIO_H

namespace cura
{

/*
 * Represents a ratio between two numbers.
 *
 * This is a facade. It behaves like a double.
 */
struct Ratio
{
    /*
     * Casts a double to a Ratio instance.
     */
    Ratio(double value) : value(value / 100) {};

    /*
     * Casts the Ratio instance to a double.
     */
    operator double() const
    {
        return value;
    }

    /*
     * The actual ratio, as a double.
     */
    double value = 0;
};

}

#endif //RATIO_H