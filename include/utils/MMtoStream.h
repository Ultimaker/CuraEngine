// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "utils/string.h"

namespace cura
{

/*!
 * Struct to make it possible to inline calls to writeInt2mm with writing other stuff to the output stream
 */
struct MMtoStream
{
    int64_t value; //!< The coord in micron

    friend inline std::ostream& operator<<(std::ostream& out, const MMtoStream precision_and_input)
    {
        writeInt2mm(precision_and_input.value, out);
        return out;
    }
};

} // namespace cura
