/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSING_SAGGING_MODEL_H
#define TEXTURE_PROCESSING_SAGGING_MODEL_H

namespace cura
{

/*!
 * Model of how a layer with overhang sags over the previous layer.
 * 
 * The model employed here models the cross section of a layer as a circle.
 * Non-overhanging layers are modeled as a rectangle with half a circle attached, with radius r=layer_height/2.
 * An overhanging layer has the center of the circle moved over y downward and receding over x,
 * as well as an increased radius.
 * 
 * The rate at which x y and r change is presupposed to depend linearly on the amount of overhang.
 * 
 * The simplified model estimates the rates Cx, Cy and Cr from a single observation, namely w, which is
 * the smallest overhang at which a vertical plane appears to be purely black.
 * Cy and Cr are supposed to be equal because the top of the circle will always be at the top of the layer.
 * Cx is deduced from supposing conservation of volume.
 */
class SaggingModel
{
public:
    float Cx; //!< ratio of mm receding per mm overhangs
    float C; //!< common term in some equations

    /*!
     * \param sagging_per_overhang Ratio of mm vertical sagging per mm overhang
     */
    SaggingModel(float sagging_per_overhang)
    : Cx(1.0f - sqrt(2.0f) / sagging_per_overhang)
    , C((1 - 2*Cx + Cx*Cx))
    {
    }

    float getC()
    {
        return C;
    }

    float getCx()
    {
        return Cx;
    }
};

} // namespace cura

#endif // TEXTURE_PROCESSING_SAGGING_MODEL_H
