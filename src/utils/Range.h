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
    : min(std::numeric_limits<T>::max())
    , max(std::numeric_limits<T>::min())
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

    void include(const Range<T> vals)
    {
        min = std::min(min, vals.min);
        max = std::max(max, vals.max);
    }

    bool inside(const T val) const
    {
        return val <= max && val >= min;
    }

    bool overlap(const Range<T>& other) const
    {
        return !(min > other.max || max < other.min);
    }

    Range intersection(const Range<T>& other) const
    {
        return Range<T>(std::max(min, other.min), std::min(max, other.max));
    }

    void expand(const T amount)
    {
        min -= amount;
        max += amount;
    }

    Range expanded(const T amount) const
    {
        Range ret = *this;
        ret.expand(amount);
        return ret;
    }

    T size() const
    {
        if (max == std::numeric_limits<T>::min())
        {
            return 0;
        }
        assert(min != std::numeric_limits<T>::max() && "If range.max is set then range.min should also be set!");
        return max - min;
    }

    T middle() const
    {
        return (max + min) / static_cast<T>(2);
    }

    template<class CharT, class TraitsT>
    friend
    std::basic_ostream<CharT, TraitsT>&
    operator <<(std::basic_ostream<CharT, TraitsT>& os, const Range& r)
    {
        return os << "(" << r.min << " - " << r.max << ")";
    }

};

}//namespace cura
#endif//UTILS_RANGE_H

