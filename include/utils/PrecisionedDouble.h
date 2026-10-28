// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include "utils/string.h"

namespace curaengine
{

/*!
 * Struct to make it possible to inline calls to writeDoubleToStream with writing other stuff to the output stream
 */
struct PrecisionedDouble
{
    uint8_t precision; //!< Number of digits after the decimal mark with which to convert to string
    double value; //!< The double value

    bool wouldWriteZero() const
    {
        return (std::abs(value) * std::pow(10.0, precision)) < 1.0;
    }

    static bool equalGivenPrecision(const PrecisionedDouble& a, const PrecisionedDouble& b)
    {
        // We expect the 'value's here to be in the hundereds or thousands or so at max, so this implementation should be OK.
        return (a.value == b.value) || (std::round(a.value * std::pow(10, a.precision)) == std::round(b.value * std::pow(10, b.precision)));
    }

    friend inline std::ostream& operator<<(std::ostream& out, const PrecisionedDouble precision_and_input)
    {
        writeDoubleToStream(precision_and_input.precision, precision_and_input.value, out);
        return out;
    }
};

} // namespace curaengine
