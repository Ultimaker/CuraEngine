/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSING_MATERIAL_H
#define TEXTURE_PROCESSING_MATERIAL_H

#include "../settings/settings.h" // ColourUsage

namespace cura
{

/*!
 * The material used in a texture.
 * 
 * This class just holds the image data and has some nice utility functions.
 */
class Material
{
public:
    /*!
     * non-initializing constructor
     */
    Material();

    /*!
     * Set the pixel data of the image
     * \param data pointer to the array of data in RGBA, left-to-right, top-to-bottom
     */
    void setData(unsigned char* data);

    /*!
     * Set the dimensions of the image
     * \param width The horizontal length of the imnage
     * \param height The vertical length of the imnage
     * \param depth The number of color channels
     */
    void setDimensions(unsigned int width, unsigned int height, unsigned int depth);

    /*!
     * get the color value at a particular place in the image
     * 
     * \param x place in the horizontal direction left to right (value between zero and one)
     * \param y place in the vertical direction top to bottom (value between zero and one)
     * \param color The color channel to check
     * \return a value between zero and one
     */
    float getColor(float x, float y, ColourUsage color) const;

    /*!
     * print out something which looks like the picture through std::cerr
     * \param double_width Whether to double each character being written, so that the width is visually similar to the height of each pixel.
     */
    void debugOutput(bool double_width = true) const;
protected:
    unsigned char* data; //!< pixel data in rgb-row-first (or bgr-row first ?)
    unsigned int width, height, depth; //!< image dimensions

    /*!
     * Get a color value from the data
     * \param x place in the horizontal direction left to right (value between zero and one)
     * \param y place in the vertical direction top to bottom (value between zero and one)
     * \return the color data (0-256)
     */
    float getColorData(float x, float y, unsigned int z) const;
};

} // namespace cura

#endif // TEXTURE_PROCESSING_MATERIAL_H