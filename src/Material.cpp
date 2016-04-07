/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */

#include <limits> // numeric limits

#include "Material.h"


namespace cura
{

void Material::setData(unsigned char* data)
{
    this->data = data;
}

void Material::setWidthHeight(int width, int height)
{
    this->width = width;
    this->height = height;
}

float Material::getColor(float x, float y)
{
    int w_idx = std::max(0, std::min(x * width, width - 1));
    int h_idx = std::max(0, std::min(y * height, height - 1));
    unsigned char r = data[(h_idx * width + w_idx) * 3];
    return (float) r / std::numeric_limits<unsigned char>::max();
}

} // namespace cura