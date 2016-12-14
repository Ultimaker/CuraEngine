/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */

#include <limits> // numeric limits
#include <algorithm> // min max

#include <iostream>
#include <cassert>

#include "Material.h"


namespace cura
{

Material::Material()
: data(nullptr)
, width(0)
, height(0)
, depth(0)
{

}


void Material::setData(unsigned char* data)
{
    this->data = data;
}

void Material::setDimensions(unsigned int width, unsigned int height, unsigned int depth)
{
    this->width = width;
    this->height = height;
    this->depth = depth;
}

float Material::getRed(float x, float y) const
{
    return getColor(x, y, 0);
}
float Material::getGreen(float x, float y) const
{
    return getColor(x, y, 1);
}
float Material::getBlue(float x, float y) const
{
    return getColor(x, y, 2);
}
float Material::getAlpha(float x, float y) const
{
    return getColor(x, y, 3);
}

float Material::getGrey(float x, float y) const
{
    unsigned int w_idx, h_idx;
    getPixelCoords(x, y, w_idx, h_idx);
    unsigned int r = getColorData(w_idx, h_idx, 0);
    unsigned int g = getColorData(w_idx, h_idx, 1);
    unsigned int b = getColorData(w_idx, h_idx, 2);
    return (float) (r + g + b) / std::numeric_limits<unsigned char>::max() / 3.0;
}


float Material::getColor(float x, float y, unsigned int z) const
{
    assert((int)z >= 0 && z < depth && "Z out of bounds!");
    unsigned int w_idx, h_idx;
    getPixelCoords(x, y, w_idx, h_idx);
    unsigned char col = getColorData(w_idx, h_idx, z);
    return (float) col / std::numeric_limits<unsigned char>::max();
}

void Material::getPixelCoords(const float x_in, const float y_in, unsigned int& x_out, unsigned int& y_out) const
{
    x_out = std::max(0u, std::min((unsigned int) (x_in * width), width - 1));
    y_out = std::max(0u, std::min((unsigned int) (y_in * height), height - 1));
}


unsigned char Material::getColorData(unsigned int x, unsigned int y, unsigned int z) const
{
    return data[(y * width + x) * depth + z];
}



} // namespace cura