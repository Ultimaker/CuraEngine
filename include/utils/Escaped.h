// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <ostream>

namespace curaengine::utils
{

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

} // namespace curaengine::utils
