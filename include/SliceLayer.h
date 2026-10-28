// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <memory>
#include <vector>

#include "SliceLayerPart.h"
#include "TopSurface.h"
#include "geometry/OpenLinesSet.h"
#include "geometry/Shape.h"
#include "utils/Coord_t.h"

namespace curaengine
{
class TextureDataProvider;

/*!
    The SlicerLayer contains all the data for a single cross section of the 3D model.
 */
class SliceLayer
{
public:
    coord_t printZ; //!< The height at which this layer needs to be printed. Can differ from sliceZ due to the raft.
    coord_t thickness; //!< The thickness of this layer. Can be different when using variable layer heights.
    std::vector<SliceLayerPart> parts; //!< An array of LayerParts which contain the actual data. The parts are printed one at a time to minimize travel outside of the 3D model.
    OpenLinesSet open_polylines; //!< A list of lines which were never hooked up into a 2D polygon. (Currently unused in normal operation)
    std::shared_ptr<TextureDataProvider> texture_data_provider_; //!< Accessor to pre-sliced texture data

    /*!
     * \brief The parts of the model that are exposed at the very top of the
     * model.
     *
     * This is filled only when the top surface is needed.
     */
    TopSurface top_surface;

    /*!
     * \brief The parts of the model that are exposed at the bottom(s) of the model.
     *
     * Note: Filled only when needed.
     */
    Shape bottom_surface;

    /*!
     * Get the all outlines of all layer parts in this layer.
     *
     * \param external_polys_only Whether to only include the outermost outline of each layer part
     * \return A collection of all the outline polygons
     */
    Shape getOutlines(bool external_polys_only = false) const;

    /*!
     * Get the all outlines of all layer parts in this layer.
     * Add those polygons to @p result.
     *
     * \param external_polys_only Whether to only include the outermost outline of each layer part
     * \param result The result: a collection of all the outline polygons
     */
    void getOutlines(Shape& result, bool external_polys_only = false) const;

    ~SliceLayer();
};

} // namespace curaengine
