//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef DURATION_H
#define DURATION_H

namespace cura
{

/*
 * \brief Represents a duration in seconds.
 *
 * This is a facade. It behaves like a double, only it can't be negative.
 */
struct Duration
{
    /*
     * \brief Default constructor setting the duration to 0.
     */
    Duration() : value(0) {};

    /*
     * \brief Casts a double to a Duration instance.
     */
    Duration(double value) : value(std::max(value, 0.0)) {};

    /*
     * \brief Casts the Duration instance to a double.
     */
    operator double() const
    {
        return value;
    };

    /*
     * Some operators to do arithmetic with Durations.
     */
    Duration operator +(const Duration& other) const
    {
        return Duration(value + other.value);
    };
    Duration operator -(const Duration& other) const
    {
        return Duration(value + other.value);
    };
    Duration& operator +=(const Duration& other)
    {
        value += other.value;
        return *this;
    }
    Duration& operator -=(const Duration& other)
    {
        value -= other.value;
        return *this;
    }

    /*
     * \brief The actual duration, as a double.
     */
    double value = 0;
};

}

#endif //DURATION_H