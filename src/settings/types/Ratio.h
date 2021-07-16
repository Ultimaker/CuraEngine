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
class Ratio
{
public:
    /*
     * \brief Default constructor setting the ratio to 1.
     */
    constexpr Ratio() : value(1.0) {};

    /*
     * \brief Casts a double to a Ratio instance.
     */
    constexpr Ratio(double value) : value(value) {};

    /*!
     * Create the Ratio with a numerator and a divisor from arbitrary types
     * \tparam E1 required to be castable to a double
     * \tparam E2 required to be castable to a double
     * \param numerator the numerator of the ratio
     * \param divisor the divisor of the ratio
     */
    template <typename E1, typename E2>
    constexpr Ratio(const E1& numerator, const E2& divisor)
        : value(static_cast<double>(numerator) / static_cast<double>(divisor)) {};

    /*
     * \brief Casts the Ratio instance to a double.
     */
    operator double() const
    {
        return value;
    }

    /*
     * Some Relational operators
     */
    template <typename E>
    constexpr bool operator==(const E& rhs) const
    {
        return value == static_cast<double>(rhs);
    }

    template <typename E>
    constexpr bool operator!=(const E& rhs) const
    {
        return !(rhs == *this);
    }

    /*
     * Some operators for arithmetic on ratios.
     */
    Ratio operator *(const Ratio& other) const
    {
        return Ratio(value * other.value);
    }
    template<typename E> Ratio operator *(const E& other) const
    {
        return Ratio(value * other);
    }
    Ratio operator /(const Ratio& other) const
    {
        return Ratio(value / other.value);
    }
    template<typename E> Ratio operator /(const E& other) const
    {
        return Ratio(value / other);
    }
    Ratio& operator *=(const Ratio& other)
    {
        value *= other.value;
        return *this;
    }
    template<typename E> Ratio& operator *=(const E& other)
    {
        value *= other;
        return *this;
    }
    Ratio& operator /=(const Ratio& other)
    {
        value /= other.value;
        return *this;
    }
    template<typename E> Ratio& operator /=(const E& other)
    {
        value /= other;
        return *this;
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
