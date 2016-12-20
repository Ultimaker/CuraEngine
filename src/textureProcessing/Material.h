/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSING_MATERIAL_H
#define TEXTURE_PROCESSING_MATERIAL_H

#include <memory> // shared_ptr

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
     * Destructor
     * 
     * deletes the image data
     */
    ~Material();

    /*!
     * Load an image from file.
     * 
     * Crash if this doesn't work. (unsupported file type, IO exception, etc.)
     */
    void loadImage(const char* filename);

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

    std::shared_ptr<unsigned char> data; //!< pixel data in rgb-row-first (or bgr-row first ?)
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