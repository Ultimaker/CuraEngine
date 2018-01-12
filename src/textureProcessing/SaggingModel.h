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
    float Cy; //!< ratio of mm falling per mm overhang
    float Cr; //!< ratio of mm increase in radius per mm overhang
    float Cx; //!< ratio of mm receding per mm overhangs

    /*!
     * \param sagging_per_overhang Ratio of mm vertical sagging per mm overhang
     */
    SaggingModel(float sagging_per_overhang)
    : Cy(0.5f * sagging_per_overhang)
    , Cr(Cy)
    , Cx(5.0 / 8.0 * M_PI * Cy)
    {
    }

    float getOcclusionOverhangRatio(float vertical_face_component, float horizontal_face_component)
    {
        return Cy * vertical_face_component - Cx * horizontal_face_component + Cr;
    }
};

} // namespace cura

#endif // TEXTURE_PROCESSING_SAGGING_MODEL_H
