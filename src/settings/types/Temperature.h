//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

namespace cura
{

/*
 * \brief Represents a temperature in degrees Celsius.
 *
 * This is a facade. It behaves like a double.
 */
struct Temperature
{
    /*
     * \brief Default constructor setting the temperature to 0.
     */
    Temperature() : value(0.0) {};

    /*
     * \brief Casts a double to a Temperature instance.
     */
    Temperature(double value) : value(value) {};

    /*
     * \brief Casts the Temperature instance to a double.
     */
    operator double() const
    {
        return value;
    }

    /*
     * Some operators to do arithmetic with Temperatures.
     */
    Temperature operator +(const Temperature& other) const
    {
        return Temperature(value + other.value);
    };
    Temperature operator -(const Temperature& other) const
    {
        return Temperature(value - other.value);
    };
    Temperature& operator +=(const Temperature& other)
    {
        value += other.value;
        return *this;
    }
    Temperature& operator -=(const Temperature& other)
    {
        value -= other.value;
        return *this;
    }

    /*
     * \brief The actual temperature, as a double.
     */
    double value = 0;
};

}

#endif //TEMPERATURE_H