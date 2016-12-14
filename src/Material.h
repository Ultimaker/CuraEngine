/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef MATERIAL_H
#define MATERIAL_H

namespace cura
{

class Material
{
public:
    void setData(unsigned char* data);
    void setDimensions(int width, int height, int depth);
    /*!
     * get some value representing the getColor
     * 
     * red?
     * 
     * TODO
     * 
     * \return a value between zero and one
     */
    float getColor(float x, float y) const;
protected:
    unsigned char* data; //!< pixel data in rgb-row-first (or bgr-row first ?)
    int width, height, depth; //!< image dimensions
};

} // namespace cura

#endif // MATERIAL_H