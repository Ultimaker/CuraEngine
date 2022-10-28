// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_STRING_H
#define UTILS_STRING_H

#include <array>
#include <cassert>
#include <ostream>
#include <span>

#include "Coord_t.h"
#include "math.h"

namespace cura
{

// c++11 no longer supplies a strcasecmp, so define our own version.
static inline int stringcasecompare(const char* a, const char* b)
{
    while (*a && *b)
    {
        if (tolower(*a) != tolower(*b))
            return tolower(*a) - tolower(*b);
        a++;
        b++;
    }
    return *a - *b;
}

/*!
 * \brief Format a fixed point unsigned integer as a decimal ascii string.
 * \tparam fixed_precision log10 of the fixed point scaling factor
 * \tparam out_precision Number of decimal places to be written
 * \param buffer_end Pointer past the end of the output buffer
 * \param value The unsigned value to format
 * \return A pair of char*, delimiting the result range in the buffer
 */
template<unsigned fixed_precision, unsigned out_precision, size_t buffer_length>
static inline std::span<char> format_unsigned_decimal_fixed_point(std::array<char, buffer_length>& buffer, size_t value)
{
    // Rounding to the required precision
    if constexpr (fixed_precision > out_precision)
    {
        value = round_divide(value, ipow(size_t{ 10 }, fixed_precision - out_precision));
    }

    if (value != 0)
    {
        // Writes pairs of digit backward in the buffer. This is heavily inspired by the fmt library.
        size_t quotient = value;
        auto begin = buffer.end();
        while (quotient >= 10)
        {
            const char* twodigits = &"0001020304050607080910111213141516171819"
                                     "2021222324252627282930313233343536373839"
                                     "4041424344454647484950515253545556575859"
                                     "6061626364656667686970717273747576777879"
                                     "8081828384858687888990919293949596979899"[(quotient % 100) * 2];
            quotient /= 100;
            begin -= 2;
            assert(begin >= buffer.begin());
            begin[0] = twodigits[0];
            begin[1] = twodigits[1];
        }
        if (quotient > 0)
        { // Odd number of digits
            assert(begin > buffer.begin());
            *--begin = '0' + static_cast<char>(quotient);
        }
        else if (*begin == '0')
        { // The last division by 100 might have produced a leading zero, drop it
            begin++;
        }
        assert(*begin != '0');

        // Trims zeros at the right of the fractional part
        const auto first_integral_place = buffer.end() - out_precision;
        auto end = buffer.end();
        while (end > first_integral_place)
        {
            if (*(end - 1) != '0')
            { // Stop at the first non '0' (will break)
                break;
            }
            else
            {
                --end; // Trims the rightmost '0'
            }
        }
        if (end > first_integral_place)
        { // We have a factional part
            if (begin < first_integral_place)
            {
                // We also have an integral part: move it one char to the left to make room for the '.'
                assert(begin > buffer.begin());
                *std::copy(begin, first_integral_place, begin - 1) = '.';
                --begin;
            }
            else
            {
                // fractional only: fill '0's at the left of the fractional part
                std::fill(first_integral_place, begin, '0');
                // In theory, a leading zero before the decimal point should not required by Gcode, however omitting it crashes PrusaSlicer viewer.
                begin = first_integral_place - 2;
                assert(begin >= buffer.begin());
                begin[0] = '0';
                begin[1] = '.';
            }
        }
        assert(*begin != '0' || begin[1] == '.');
        assert(*begin != '.');
        return { begin, end };
    }
    else
    {
        static_assert(buffer_length >= 1);
        char* const begin = buffer.end() - 1;
        *begin = '0';
        return { begin, buffer.end() };
    }
}

/*!
 * \brief Format a fixed point signed integer as a decimal ascii string.
 * \tparam fixed_precision log10 of the fixed point scaling factor
 * \tparam out_precision Number of decimal places to be written
 * \param buffer_end Pointer past the end of the output buffer
 * \param value The unsigned value to format
 * \return A pair of char*, delimiting the result range in the buffer
 */
template<unsigned fixed_precision, unsigned out_precision, size_t buffer_length>
static inline std::span<char> format_signed_decimal_fixed_point(std::array<char, buffer_length>& buffer, ptrdiff_t value)
{
    bool is_neg = value < 0;
    auto abs_value = static_cast<size_t>(is_neg ? -value : value);
    auto result = format_unsigned_decimal_fixed_point<fixed_precision, out_precision>(buffer, abs_value);
    if (is_neg)
    {
        auto begin = --result.begin();
        *begin = '-';
        return { begin, result.end() };
    }
    return result;
}

/*!
 * \brief Wraps a coord_t for writing it in base 10 to a ostream.
 */
struct MMtoStream
{
    coord_t value; //!< The coord in micron

    friend inline std::ostream& operator<<(std::ostream& out, const MMtoStream precision_and_input)
    {
        constexpr size_t buffer_size = std::numeric_limits<coord_t>::digits10 + 3;
        // +1 because digits10 is log_10(max) rounded down
        // +1 for the sign
        // +1 for the decimal separator
        std::array<char, buffer_size> buffer;
        auto res = format_signed_decimal_fixed_point<INT10POW_PER_MM, INT10POW_PER_MM>(buffer, precision_and_input.value);
        out.write(&res.front(), res.size());
        return out;
    }
};

/*!
 * \brief Wraps a double for writing it in base 10 to a ostream with a fixed number of decimal places.
 * \tparam precision The number of decimal places
 * \warning Does not allows values larger that LLONG_MAX / 10^precision
 */
template<size_t precision>
struct PrecisionedDouble
{
    // unsigned int precision; //!< Number of digits after the decimal mark with which to convert to string
    double value; //!< The double value

    friend inline std::ostream& operator<<(std::ostream& out, const PrecisionedDouble input)
    {
        constexpr size_t buffer_size = std::numeric_limits<coord_t>::digits10 + 3;
        std::array<char, buffer_size> buffer;

        constexpr size_t factor = ipow(size_t{ 10 }, precision);
        double scaled_value = input.value * factor;
        assert(std::abs(scaled_value) < static_cast<double>(std::numeric_limits<coord_t>::max()));
        coord_t fixed_point = std::llround(scaled_value);
        auto res = format_signed_decimal_fixed_point<precision, precision>(buffer, fixed_point);
        out.write(&res.front(), res.size());
        return out;
    }
};

/*!
 * Struct for writing a string to a stream in an escaped form
 */
struct Escaped
{
    const char* str;

    /*!
     * Streaming function which replaces escape sequences with extra slashes
     */
    friend inline std::ostream& operator<<(std::ostream& os, const Escaped& e)
    {
        for (const char* char_p = e.str; *char_p != '\0'; char_p++)
        {
            switch (*char_p)
            {
            case '\a':
                os << "\\a";
                break;
            case '\b':
                os << "\\b";
                break;
            case '\f':
                os << "\\f";
                break;
            case '\n':
                os << "\\n";
                break;
            case '\r':
                os << "\\r";
                break;
            case '\t':
                os << "\\t";
                break;
            case '\v':
                os << "\\v";
                break;
            case '\\':
                os << "\\\\";
                break;
            case '\'':
                os << "\\'";
                break;
            case '\"':
                os << "\\\"";
                break;
            case '\?':
                os << "\\\?";
                break;
            default:
                os << *char_p;
            }
        }
        return os;
    }
};

} // namespace cura

#endif // UTILS_STRING_H
