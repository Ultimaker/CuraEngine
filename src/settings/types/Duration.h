//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef DURATION_H
#define DURATION_H

#include <ostream>

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
    constexpr Duration() : value(0) {};

    /*
     * \brief Casts a double to a Duration instance.
     */
    constexpr Duration(double value) : value(value > 0.0 ? value : 0.0) {};

    /*
     * \brief Casts the Duration instance to a double.
     */
    constexpr operator double() const
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
        return Duration(value - other.value);
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

constexpr Duration operator "" _s(const long double seconds)
{
    return Duration(seconds);
}


inline std::ostream& operator<< (std::ostream& out, const Duration seconds)
{
    constexpr bool pretty_print = false;

    double s = seconds;
    if (pretty_print && seconds > 60)
    {
        int min = seconds / 60;
        s -= min * 60;
        if (min > 60)
        {
            int hrs = min / 60;
            min -= hrs * 60;
            out << hrs << "h ";
        }
        out << min << "min ";
    }
    out << s << (pretty_print ? "s" : "");
    return out;
}

}

#endif //DURATION_H