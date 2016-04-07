/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */

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


} // namespace cura