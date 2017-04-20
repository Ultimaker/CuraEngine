/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_SPAGHETTI_INFILL_H
#define INFILL_SPAGHETTI_INFILL_H

#include <list>

#include "../utils/intpoint.h"
#include "../utils/polygon.h"
#include "../sliceDataStorage.h"

namespace cura {

/*!
 * Spaghetti infill is a type of infill which fills every so many layers, but extrudes as much filament corresponding to the total unfilled volume under the filling area.
 *
 * A filling layer is inserted when a a pillar of infill areas is becoming too high, or when the angle between the filling areas is too shallow.
 *
 * The filling area might be smaller than the actual infill area, so that we fill the pillar from a smaller top area.
 *
 * Infill pillars can join each other if they are connected on the top. The total volume will then be extruded from the top.
 *
 * Where the model spits into two from bottom to top, one of the top pieces will be connected to the lower part as one big pillar, while a new pillar will be generated for the other top part.
 * Which part the base will be connected to is arbitrary.
 *
 */
class SpaghettiInfill
{
public:
    /*!
     * Generate the filling areas and corresponding volume to extrude over such areas for spaghetti infill.
     */
    static void generateSpaghettiInfill(SliceMeshStorage& mesh);

protected:
    class InfillPillar
    {
    public:
        SliceLayerPart* top_slice_layer_part = nullptr; //!< A reference to the slice_layer_part from which the top part is generated
        PolygonsPart top_part; //!< The top area of this pillar
        double total_volume_mm3; //!< The total volume of the pillar
        coord_t connection_inset_dist; //!< Horizontal component of the spaghetti_max_infill_angle: the distance insetted corresponding to the maximum angle which can be filled by spaghetti infill.
        coord_t bottom_z; //!< The z coordinate of the bottom of the first layer this pillar is present in
        int last_layer_added = -1; //!< The last layer from which areas got added to this pillar

        /*!
         * Basic constructor of a pillar from a single area, which is to be the top of the new pillar
         *
         * \param _top_part The area which is the base and the top of the new pillar
         * \param connection_inset_dist Horizontal component of the spaghetti_max_infill_angle
         * \param layer_height The layer height of the layer which contains the \p _top_part
         * \param bottom_z The z coordinate of the bottom of layer which contains \p _top_part
         */
        InfillPillar(const PolygonsPart& _top_part, coord_t connection_inset_dist, int layer_height, coord_t bottom_z)
        : top_part(_top_part) // TODO: prevent copy construction! Is that possible?
        , total_volume_mm3(INT2MM(INT2MM(top_part.area())) * INT2MM(layer_height))
        , connection_inset_dist(connection_inset_dist)
        , bottom_z(bottom_z)
        {
        }

        /*!
         * Check whether the top of this pillar is connected (enough) to the given \p infill_part.
         * It is assumed the infill_part is on the layer directly above the top part of this pillar.
         *
         * \param infill_part The part to check for connectivity
         * \return Whether the infill part can be incorporated in this pillar
         */
        bool isConnected(const PolygonsPart& infill_part) const;

        /*!
         * Register the volume of this infill pillar in the sliceDataStorage.
         * The filling area and the volume are saved in \ref SliceLayerPart::spaghetti_infill_volumes
         *
         * Note that the filling area is different from the infill area, because the spaghetti can curl toward the sides.
         *
         * \param filling_area_inset The inset from the boundary of the walls to get from the infill area to the filling area
         * \param line_width The line width used to generate an area just large enough for infill lines to be generated, when the infill area would otherwise be too small to get infill
         */
        void addToTopSliceLayerPart(coord_t filling_area_inset, coord_t line_width);
    };
private:
    /*!
     * Add an area to the pillar base:
     * - add it to an existing pillar if possible
     * - otherwise create a new pillar for this area
     * The pillar to which the area was added is returned
     *
     * \param infill_part The area to add to the base
     * \param pillar_base The collection of pillars used up till the current layer
     * \param connection_inset_dist The distance insetted corresponding to the maximum angle which can be filled by spaghetti infill
     * \param layer_height The layer height of the added area
     * \param bottom_z The z coordinate of the bottom of the layer which contains the \p infill_part
     */
    static InfillPillar& addPartToPillarBase(const PolygonsPart& infill_part, std::list<InfillPillar>& pillar_base, coord_t connection_inset_dist, int layer_height, coord_t bottom_z);
};

}//namespace cura

#endif//INFILL_SPAGHETTI_INFILL_H
