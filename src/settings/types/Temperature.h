//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

namespace cura
{

/*
 * Represents a temperature in degrees Celsius.
 *
 * This is a facade. It behaves like a double.
 */
struct Temperature
{
    /*
     * Casts a double to a Temperature instance.
     */
    Temperature(double value) : value(value) {};

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

#endif //TEMPERATURE_H