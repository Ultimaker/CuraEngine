/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_MATH_H
#define UTILS_MATH_H

namespace cura
{


template<typename T> inline T square(const T& a) { return a * a; }

inline int round_divide(int dividend, int divisor) //!< Return dividend divided by divisor rounded to the nearest integer
{
    return (dividend + divisor / 2) / divisor;
}
inline int round_up_divide(int dividend, int divisor) //!< Return dividend divided by divisor rounded to the nearest integer
{
    return (dividend + divisor - 1) / divisor;
}

}//namespace cura
#endif // UTILS_MATH_H

