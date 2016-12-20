/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSING_TEXTURE_PROXIMITY_PROCESSOR_H
#define TEXTURE_PROCESSING_TEXTURE_PROXIMITY_PROCESSOR_H

#include <vector>

#include "../utils/intpoint.h"
#include "../utils/SparseLineGrid.h"

#include "../settings/settings.h"

#include "../slicer/SlicerSegment.h"
#include "TexturedMesh.h"

namespace cura
{

/*!
 * Class for recording texture coordinates at places where textures are defined, for later looking in the proximity of a texture.
 */
class TextureProximityProcessor
{
public:
    /*!
     * Helper class to retrieve and store texture to bump map settings
     */
    struct Settings
    {
        coord_t proximity; //!< The distance within which to search for nearby texture
        Settings(coord_t proximity)
        : proximity(proximity)
        {
        }
    };
    /*!
     * default constructor
     * 
     * initializes the \ref SparseGrid::cell_size of \ref TextureProximityProcessor::loc_to_slice
     * 
     * \param settings The settings with which to \ref TextureProximityProcessor::processBumpMap
     */
    TextureProximityProcessor(const Settings settings, unsigned int slice_layer_count);

    /*!
     * Register that a particular face was sliced to a particular texture segment.
     * \param face_segment The geometrical segment of the face
     * \param texture_segment The corresponding texture coordinates
     * \param layer_nr The layer for which to register a face being sliced
     */
    void registerTexturedFaceSlice(SlicerSegment face_segment, MatSegment texture_segment, unsigned int layer_nr);

    /*!
     * 
     * \param default_color Default color where no texture is present
     */
    float getColor(const Point location, const unsigned int layer_nr, ColourUsage color, float default_color);
protected:
    /*!
     * A sliced segment in combination with the corresponding texture slice.
     */
    struct TexturedFaceSlice
    {
        SlicerSegment face_segment;
        MatSegment mat_segment;
    };

    /*!
     * Locator to find the line segment of a \ref TexturedFaceSlice
     */
    struct TexturedFaceSliceLocator
    {
        std::pair<Point, Point> operator()(const TexturedFaceSlice& elem) const
        {
            return std::make_pair(elem.face_segment.start, elem.face_segment.end);
        }
    };

    /*!
     * The settings with which to \ref TextureBumpMapProcessor::processBumpMap
     */
    Settings settings;

    /*!
     * A grid to efficiently look op which texture segment best fits the slicer segment.
     * 
     * A vector of elements for each layer
     */
    std::vector<SparseLineGrid<TexturedFaceSlice, TexturedFaceSliceLocator>> loc_to_slice;
};

} // namespace cura

#endif // TEXTURE_PROCESSING_TEXTURE_PROXIMITY_PROCESSOR_H