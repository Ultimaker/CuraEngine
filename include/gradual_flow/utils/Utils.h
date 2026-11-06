// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <cmath>
#include <stdexcept>
#include <tuple>


namespace curaengine::gradual_flow::utils
{

/*
 * \brief Converts HSV values to RGB values.
 *
 * \param h Hue value in range [0, 360]
 * \param saturation Saturation value in range [0, 100]
 * \param value Value value in range [0, 100]
 *
 * \return A tuple containing the RGB values in range [0, 255]
 */
std::tuple<int, int, int> hsvToRgb(double h, double saturation, double value)
{
    // Code taken from https://www.codespeedy.com/hsv-to-rgb-in-cpp/ and slightly modified
    if (h > 360. || h < 0. || saturation > 100. || saturation < 0. || value > 100. || value < 0.)
    {
        throw std::invalid_argument("The given HSV values are not in valid range");
    }
    auto s = saturation * .01;
    auto v = value * .01;
    auto c = s * v;
    auto x = c * (1. - abs(fmod(h / 60.0, 2.) - 1.));
    auto m = v - c;
    auto r = 0., g = 0., b = 0.;
    if (h >= 0. && h < 60.)
    {
        r = c;
        g = x;
        b = 0.;
    }
    else if (h >= 60. && h < 120.)
    {
        r = x;
        g = c;
        b = 0.;
    }
    else if (h >= 120. && h < 180.)
    {
        r = 0.;
        g = c;
        b = x;
    }
    else if (h >= 180. && h < 240.)
    {
        r = 0.;
        g = x;
        b = c;
    }
    else if (h >= 240. && h < 300.)
    {
        r = x;
        g = 0.;
        b = c;
    }
    else
    {
        r = c;
        g = 0.;
        b = x;
    }

    int red = (r + m) * 255.;
    int green = (g + m) * 255.;
    int blue = (b + m) * 255.;

    return std::make_tuple(red, green, blue);
}

} // namespace curaengine::gradual_flow::utils
