/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef MATERIAL_H
#define MATERIAL_H

namespace cura
{

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
     * get the red value at a particular place in the image
     * 
     * \param x place in the horizontal direction left to right (value between zero and one)
     * \param y place in the vertical direction top to bottom (value between zero and one)
     * \return a value between zero and one
     */
    float getRed(float x, float y) const;
    /*!
     * get the green value at a particular place in the image
     * 
     * \param x place in the horizontal direction left to right (value between zero and one)
     * \param y place in the vertical direction top to bottom (value between zero and one)
     * \return a value between zero and one
     */
    float getGreen(float x, float y) const;
    /*!
     * get the blue value at a particular place in the image
     * 
     * \param x place in the horizontal direction left to right (value between zero and one)
     * \param y place in the vertical direction top to bottom (value between zero and one)
     * \return a value between zero and one
     */
    float getBlue(float x, float y) const;
    /*!
     * get the alpha value at a particular place in the image
     * 
     * \param x place in the horizontal direction left to right (value between zero and one)
     * \param y place in the vertical direction top to bottom (value between zero and one)
     * \return a value between zero and one
     */
    float getAlpha(float x, float y) const;
    /*!
     * get the R/G/B/A value at a particular place in the image
     * 
     * for z:
     * 0=red
     * 1=green
     * 2=blue
     * 3=alpha
     * 
     * \param x place in the horizontal direction left to right (value between zero and one)
     * \param y place in the vertical direction top to bottom (value between zero and one)
     * \param z The color channel to check
     * \return a value between zero and one
     */
    float getColor(float x, float y, unsigned int z) const;
    /*!
     * get the grey value at a particular place in the image
     * 
     * \param x place in the horizontal direction left to right (value between zero and one)
     * \param y place in the vertical direction top to bottom (value between zero and one)
     * \return a value between zero and one
     */
    float getGrey(float x, float y) const;
protected:
    unsigned char* data; //!< pixel data in rgb-row-first (or bgr-row first ?)
    unsigned int width, height, depth; //!< image dimensions

    /*!
     * Get a color value from the data
     * \param x place in the horizontal direction left to right (value between zero and one)
     * \param y place in the vertical direction top to bottom (value between zero and one)
     * \return the color data (0-256)
     */
    unsigned char getColorData(unsigned int x, unsigned int y, unsigned int z) const;

    /*!
     * Convert float coordinates to pixel coordinates
     * 
     * \param[in] x_in place in the horizontal direction left to right (value between zero and one)
     * \param[in] y_in place in the vertical direction top to bottom (value between zero and one)
     * \param[out] x_out The x component of the pixel location
     * \param[out] y_out The y component of the pixel location
     */
    void getPixelCoords(const float x_in, const float y_in, unsigned int& x_out, unsigned int& y_out) const;
};

} // namespace cura

#endif // MATERIAL_H