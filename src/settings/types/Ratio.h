//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef RATIO_H
#define RATIO_H

namespace cura
{

/*
 * \brief Represents a ratio between two numbers.
 *
 * This is a facade. It behaves like a double.
 */
struct Ratio
{
    /*
     * \brief Default constructor setting the ratio to 1.
     */
    constexpr Ratio() : value(1.0) {};

    /*
     * \brief Casts a double to a Ratio instance.
     */
    constexpr Ratio(double value) : value(value / 100) {};

    /*
     * \brief Casts the Ratio instance to a double.
     */
    operator double() const
    {
        return value;
    }

    /*
     * \brief The actual ratio, as a double.
     */
    double value = 0;
};

constexpr Ratio operator "" _r(const long double ratio)
{
    return Ratio(ratio);
}

}

#endif //RATIO_H