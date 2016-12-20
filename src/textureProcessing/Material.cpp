/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */

#include <limits> // numeric limits
#include <algorithm> // min max

#include <iostream>
#include <cassert>


#include "Material.h"

#include "stb/stb_image.h"


namespace cura
{
/*!
 * custom destructor for the data to be used by the shared_pointer
 */
struct ArrayDeleter
{
    void operator ()(unsigned char* p)
    { 
        stbi_image_free(p); 
    }
};

Material::Material()
: data(nullptr, ArrayDeleter())
, width(0)
, height(0)
, depth(0)
{

}

Material::~Material()
{
}



void Material::setData(unsigned char* data)
{
    this->data = std::shared_ptr<unsigned char>(data);
}

void Material::setDimensions(unsigned int width, unsigned int height, unsigned int depth)
{
    this->width = width;
    this->height = height;
    this->depth = depth;
}

float Material::getColor(float x, float y, ColourUsage color) const
{
    assert(x >= 0.0f && x <= 1.0f);
    assert(y >= 0.0f && y <= 1.0f);
    switch (color)
    {
        case ColourUsage::RED:
        case ColourUsage::GREEN:
        case ColourUsage::BLUE:
        case ColourUsage::ALPHA:
        {
            assert((int)color >= 0 && (unsigned int)color < depth && "Z out of bounds!");
            return getColorData(x, y, (unsigned int) color);
        }
        case ColourUsage::GREY:
        default:
        {
            float r = getColorData(x, y, (unsigned int) ColourUsage::RED);
            float g = getColorData(x, y, (unsigned int) ColourUsage::GREEN);
            float b = getColorData(x, y, (unsigned int) ColourUsage::BLUE);
            return (r + g + b) / 3.0;
        }
    }
}


float Material::getColorData(float x, float y, unsigned int z) const
{
    unsigned int x_idx = (unsigned int) (x * (width - 1) + 0.5);
    assert(x_idx >= 0 && x_idx < width && "requested X is out of bounds!");
    unsigned int y_idx = (unsigned int) (y * (height - 1) + 0.5);
    assert(y_idx >= 0 && y_idx < height && "requested Y is out of bounds!");

    unsigned char col = data[(y_idx * width + x_idx) * depth + z];
    return (float) col / std::numeric_limits<unsigned char>::max();
}




void Material::debugOutput(bool dw) const
{
    std::cerr << "\nImage size: " << width << " x " << height << " (" << depth << "channels)\n";
    std::cerr << '+';
    for (unsigned int i = 0; i < width; i++)
    {
        std::cerr << ((dw)? "--" : "-");
    }
    std::cerr << "+\n";
    for (unsigned int y = 0; y < height; y++)
    {
        std::cerr << "|";
        for (unsigned int x = 0; x < width; x++)
        {
            int val = (data[(y*width+x)*depth] * 10 / 256);

            switch (val)
            {
                case 0:
                    std::cerr << ((dw)? "  " : " ");
                    break;
                case 1:
                    std::cerr << ((dw)? ".." : ".");
                    break;
                case 2:
                    std::cerr << ((dw)? ",," : ",");
                    break;
                case 3:
                    std::cerr << ((dw)? "::" : ":");
                    break;
                case 4:
                    std::cerr << ((dw)? ";;" : ";");
                    break;
                case 5:
                    std::cerr << ((dw)? "++" : "+");
                    break;
                case 6:
                    std::cerr << ((dw)? "░░" : "░");
                    break;
                case 7:
                    std::cerr << ((dw)? "▒▒" : "▒");
                    break;
                case 8:
                    std::cerr << ((dw)? "▓▓" : "▓");
                    break;
                default:
                    if (val > 8)
                    {
                        std::cerr << ((dw)? "██" : "█");
                    }
                    else
                    {
                        std::cerr << ((dw)? "  " : " ");
                    }
            }
        }
        std::cerr << "|\n";
    }
    std::cerr << '+';
    for (unsigned int i = 0; i < width; i++)
    {
        std::cerr << ((dw)? "--" : "-");
    }
    std::cerr << "+\n";
}

} // namespace cura