/** Copyright (C) 2018 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_RANGE_H
#define UTILS_RANGE_H

#include <type_traits>
#include <limits>


namespace cura
{


/*!
 * A range of numbers
 * 
 * \tparam T the type of number
 */
template<
    typename T, //real type
    typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type
>
class Range
{
public:
    T min, max;

    Range()
    : min(std::numeric_limits<T>::max)
    , max(std::numeric_limits<T>::min)
    {}

    Range(const T min, const T max)
    : min(min)
    , max(max)
    {}

    void include(const T val)
    {
        min = std::min(min, val);
        max = std::max(max, val);
    }

    bool inside(const T val) const
    {
        return val <= max && val >= min;
    }

    bool overlap(const Range<T>& other) const
    {
        return !(min > other.max || max < other.min);
    }

    Range expand(const T amount)
    {
        min -= amount;
        max += amount;
        return *this;
    }

    T size() const
    {
        return max - min;
    }
};

}//namespace cura
#endif//UTILS_RANGE_H

