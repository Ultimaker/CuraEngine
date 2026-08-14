// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

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
    constexpr Duration()
        : value_(0)
    {
    }

    /*
     * \brief Casts a double to a Duration instance.
     */
    constexpr Duration(double value)
        : value_(value > 0.0 ? value : 0.0)
    {
    }

    /*
     * \brief Casts the Duration instance to a double.
     */
    constexpr operator double() const
    {
        return value_;
    }

    /*
     * Some operators to do arithmetic with Durations.
     */
    Duration operator+(const Duration& other) const
    {
        return Duration(value_ + other.value_);
    }

    Duration operator-(const Duration& other) const
    {
        return Duration(value_ - other.value_);
    }

    Duration& operator+=(const Duration& other)
    {
        value_ += other.value_;
        return *this;
    }

    Duration& operator-=(const Duration& other)
    {
        value_ -= other.value_;
        return *this;
    }

    /*
     * \brief The actual duration, as a double.
     */
    double value_ = 0;
};

constexpr Duration operator""_s(const long double seconds)
{
    return Duration(static_cast<double>(seconds));
}


inline std::ostream& operator<<(std::ostream& out, const Duration seconds)
{
    constexpr bool pretty_print = false;

    double s = seconds;
    if (pretty_print && seconds > 60)
    {
        int min = static_cast<int>(seconds) / 60;
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

inline Duration operator*(const double lhs, const Duration& rhs)
{
    return Duration(lhs * rhs.value_);
}

} // namespace cura

#endif // DURATION_H
