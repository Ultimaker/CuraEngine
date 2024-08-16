// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GRADUAL_FLOW_UTILS_H
#define GRADUAL_FLOW_UTILS_H


namespace cura::gradual_flow::utils
{

enum Direction
{
    Forward,
    Backward,
};

/*
 * \brief Converts HSV values to RGB values.
 *
 * \param H Hue value in range [0, 360]
 * \param S Saturation value in range [0, 100]
 * \param V Value value in range [0, 100]
 *
 * \return A tuple containing the RGB values in range [0, 255]
 */
std::tuple<int, int, int> hsvToRgb(double H, double S, double V)
{
    // Code taken from https://www.codespeedy.com/hsv-to-rgb-in-cpp/ and slightly modified
    if (H > 360. || H < 0. || S > 100. || S < 0. || V > 100. || V < 0.)
    {
        throw std::invalid_argument("The given HSV values are not in valid range");
    }
    auto s = S * .01;
    auto v = V * .01;
    auto C = s * v;
    auto X = C * (1. - abs(fmod(H / 60.0, 2.) - 1.));
    auto m = v - C;
    auto r = 0., g = 0., b = 0.;
    if (H >= 0. && H < 60.)
    {
        r = C;
        g = X;
        b = 0.;
    }
    else if (H >= 60. && H < 120.)
    {
        r = X;
        g = C;
        b = 0.;
    }
    else if (H >= 120. && H < 180.)
    {
        r = 0.;
        g = C;
        b = X;
    }
    else if (H >= 180. && H < 240.)
    {
        r = 0.;
        g = X;
        b = C;
    }
    else if (H >= 240. && H < 300.)
    {
        r = X;
        g = 0.;
        b = C;
    }
    else
    {
        r = C;
        g = 0.;
        b = X;
    }

    int R = (r + m) * 255.;
    int G = (g + m) * 255.;
    int B = (b + m) * 255.;

    return std::make_tuple(R, G, B);
}

} // namespace cura::gradual_flow::utils

#endif // GRADUAL_FLOW_UTILS_H
