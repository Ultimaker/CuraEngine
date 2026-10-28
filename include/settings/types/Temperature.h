// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

namespace curaengine
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
    constexpr Temperature()
        : value(0.0) {};

    /*
     * \brief Casts a double to a Temperature instance.
     */
    constexpr Temperature(double value_t)
        : value(value_t) {};

    /*
     * \brief Casts the Temperature instance to a double.
     */
    constexpr operator double() const
    {
        return value;
    }

    /*
     * Some operators to do arithmetic with Temperatures.
     */
    Temperature operator+(const Temperature& other) const
    {
        return Temperature(value + other.value);
    };
    Temperature operator-(const Temperature& other) const
    {
        return Temperature(value - other.value);
    };
    Temperature& operator+=(const Temperature& other)
    {
        value += other.value;
        return *this;
    }
    Temperature& operator-=(const Temperature& other)
    {
        value -= other.value;
        return *this;
    }

    /*
     * \brief The actual temperature, as a double.
     */
    double value = 0;
};

} // namespace curaengine
