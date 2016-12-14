/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */

#include <limits> // numeric limits
#include <algorithm> // min max

#include <iostream>

#include "Material.h"


namespace cura
{

void Material::setData(unsigned char* data)
{
    this->data = data;
}

void Material::setDimensions(int width, int height, int depth)
{
    this->width = width;
    this->height = height;
    this->depth = depth;
}

float Material::getColor(float x, float y) const
{
    int w_idx = std::max(0, std::min(int (x * width), width - 1));
    int h_idx = std::max(0, std::min(int (y * height), height - 1));
    unsigned char r = data[(h_idx * width + w_idx) * depth];
    return (float) r / std::numeric_limits<unsigned char>::max();
}

} // namespace cura