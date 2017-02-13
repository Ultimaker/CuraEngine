/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef INFILL_SPAGHETTI_INFILL_H
#define INFILL_SPAGHETTI_INFILL_H

#include <list>

#include "../utils/intpoint.h"
#include "../utils/polygon.h"
#include "../sliceDataStorage.h"

namespace cura {

class SpaghettiInfill
{
public:
    /*!
     * TODO
     */
    static void generateSpaghettiInfill(SliceMeshStorage& mesh);

protected:
    struct InfillPillar
    {
        SliceLayerPart* top_slice_layer_part = nullptr;
        PolygonsPart top_part;
        double total_area_mm2;
        coord_t connection_inset_dist; //!< The distance insetted corresponding to the maximum angle which can be filled by spaghetti infill.
        int layer_count = 0;
        int last_layer_added = -1;

        /*!
        * TODO
        */
        InfillPillar(PolygonsPart& _top_part, coord_t connection_inset_dist)
        : top_part(_top_part)
        , total_area_mm2(INT2MM(INT2MM(top_part.area())))
        , connection_inset_dist(connection_inset_dist)
        {
        }

        /*!
        * TODO
        */
        bool isConnected(const PolygonsPart& infill_part);

        /*!
         * TODO
         * 
         * \param line_width The line width used to generate an area just large enough for infill lines to be generated, when the infill area would otherwise be too small to get infill
         */
        void addToTopSliceLayerPart(double layer_height_mm, coord_t filling_area_inset, coord_t line_width);
    };
private:
    /*!
    * TODO
    * \param connection_inset_dist The distance insetted corresponding to the maximum angle which can be filled by spaghetti infill
    */
    static InfillPillar& addPartToPillarBase(PolygonsPart& infill_part, std::list<InfillPillar>& pillar_base, coord_t connection_inset_dist);
};

}//namespace cura

#endif//INFILL_SPAGHETTI_INFILL_H
