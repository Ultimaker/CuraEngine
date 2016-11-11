#ifndef UTILS_STRING_H
#define UTILS_STRING_H

#include <ctype.h>
#include <cstdio> // sprintf
#include <sstream> // ostringstream

#include <cinttypes> // PRId64

#include "logoutput.h"

namespace cura
{
    
//c++11 no longer supplies a strcasecmp, so define our own version.
static inline int stringcasecompare(const char* a, const char* b)
{
    while(*a && *b)
    {
        if (tolower(*a) != tolower(*b))
            return tolower(*a) - tolower(*b);
        a++;
        b++;
    }
    return *a - *b;
}

/*!
 * Efficient conversion of micron integer type to millimeter string.
 * 
 * \param coord The micron unit to convert
 * \param ss The output stream to write the string to
 */
static inline void writeInt2mm(const int64_t coord, std::ostream& ss)
{
    constexpr size_t buffer_size = 24;
    char buffer[buffer_size];
    int char_count = sprintf(buffer, "%" PRId64, coord); // convert int to string
#ifdef DEBUG
    if (char_count + 1 >= int(buffer_size)) // + 1 for the null character
    {
        logError("Cannot write %ld to buffer of size %i", coord, buffer_size);
    }
    if (char_count < 0)
    {
        logError("Encoding error while writing %ld", coord);
    }
#endif // DEBUG
    int end_pos = char_count; // the first character not to write any more
    int trailing_zeros = 1;
    while (trailing_zeros < 4 && buffer[char_count - trailing_zeros] == '0')
    {
        trailing_zeros++;
    }
    trailing_zeros--;
    end_pos = char_count - trailing_zeros;
    if (trailing_zeros == 3)
    { // no need to write the decimal dot
        buffer[char_count - trailing_zeros] = '\0';
        ss << buffer;
        return;
    }
    if (char_count <= 3)
    {
        int start = 0; // where to start writing from the buffer
        if (coord < 0)
        {
            ss << '-';
            start = 1;
        }
        ss << "0.";
        for (int nulls = char_count - start; nulls < 3; nulls++)
        { // fill up to 3 decimals with zeros
            ss << '0';
        }
        buffer[char_count - trailing_zeros] = '\0';
        ss << (static_cast<char*>(buffer) + start);
    }
    else
    {
        char prev = '.';
        int pos;
        for (pos = char_count - 3; pos <= end_pos; pos++)
        { // shift all characters and insert the decimal dot
            char next_prev = buffer[pos];
            buffer[pos] = prev;
            prev = next_prev;
        }
        buffer[pos] = '\0';
        ss << buffer;
    }
}

/*!
 * Struct to make it possible to inline calls to writeInt2mm with writing other stuff to the output stream
 */
struct MMtoStream
{
    int64_t value; //!< The coord in micron

    friend inline std::ostream& operator<< (std::ostream& out, const MMtoStream precision_and_input)
    {
        writeInt2mm(precision_and_input.value, out);
        return out;
    }
};

/*!
 * Efficient writing of a double to a stringstream
 * 
 * writes with \p precision digits after the decimal dot, but removes trailing zeros
 * 
 * \warning only works with precision up to 9 and input up to 10^14
 * 
 * \param precision The number of (non-zero) digits after the decimal dot
 * \param coord double to output
 * \param ss The output stream to write the string to
 */
static inline void writeDoubleToStream(const unsigned int precision, const double coord, std::ostream& ss)
{
    char format[5] = "%.xF"; // write a float with [x] digits after the dot
    format[2] = '0' + precision; // set [x]
    constexpr size_t buffer_size = 400;
    char buffer[buffer_size];
    int char_count = sprintf(buffer, format, coord);
#ifdef DEBUG
    if (char_count + 1 >= int(buffer_size)) // + 1 for the null character
    {
        logError("Cannot write %f to buffer of size %i", coord, buffer_size);
    }
    if (char_count < 0)
    {
        logError("Encoding error while writing %f", coord);
    }
#endif // DEBUG
    if (char_count <= 0)
    {
        return;
    }
    if (buffer[char_count - precision - 1] == '.')
    {
        int non_nul_pos = char_count - 1;
        while (buffer[non_nul_pos] == '0')
        {
            non_nul_pos--;
        }
        if (buffer[non_nul_pos] == '.')
        {
            buffer[non_nul_pos] = '\0';
        }
        else
        {
            buffer[non_nul_pos + 1] = '\0';
        }
    }
    ss << buffer;
}

/*!
 * Struct to make it possible to inline calls to writeDoubleToStream with writing other stuff to the output stream
 */
struct PrecisionedDouble
{
    unsigned int precision; //!< Number of digits after the decimal mark with which to convert to string
    double value; //!< The double value

    friend inline std::ostream& operator<< (std::ostream& out, const PrecisionedDouble precision_and_input)
    {
        writeDoubleToStream(precision_and_input.precision, precision_and_input.value, out);
        return out;
    }
};


}//namespace cura

#endif//UTILS_STRING_H
