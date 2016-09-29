#ifndef UTILS_STRING_H
#define UTILS_STRING_H

#include <ctype.h>
#include <cstdio> // sprintf
#include <sstream> // ostringstream

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
static inline void writeInt2mm(int64_t coord, std::ostream& ss)
{
    char buffer[24];
    int char_count = sprintf(buffer, "%ld", coord); // convert int to string
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
        ss << '.';
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

}//namespace cura
#endif//UTILS_STRING_H
