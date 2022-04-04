//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef VTKCOMMUNICATION_H
#define VTKCOMMUNICATION_H

#ifdef VTK_OUTPUT

#include "Communication.h" //We're implementing this interface.

namespace cura
{

class VtkCommunication : public Communication
{
public:
    //These functions have no special meaning since this communication class never starts a slice or communicate g-code.
    void beginGCode() override;
    void flushGCode() override;
    bool isSequential() const override;
    bool hasSlice() const override;
    void sendFinishedSlicing() const override;
    void sendGCodePrefix(const std::string&) const override;
    void sendLayerComplete(const LayerIndex&, const coord_t&, const coord_t&) override;
    void sendOptimizedLayerData() override;
    void sendPrintTimeMaterialEstimates() const override;
    void sendProgress(const float&) const override;
    void sliceNext() override;

    //Communication output that may be visualized in the VTK output.
    void setExtruderForSend(const ExtruderTrain& extruder) override;
    void setLayerForSend(const LayerIndex& layer_nr) override;
    void sendCurrentPosition(const Point&) override;
    void sendLineTo(const PrintFeatureType& feature_type, const Point& to, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity) override;
    void sendPolygon(const PrintFeatureType& feature_type, const ConstPolygonRef& polygon, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity) override;
    void sendPolygons(const PrintFeatureType& feature_type, const Polygons& polygons, const coord_t& line_width, const coord_t& line_thickness, const Velocity& velocity) override;

    /*!
     * Send a preview of where the structures will be generated through this
     * communication channel.
     *
     * This will get visualised in the VTK output.
     * \param outline The outline of the shape of a structure.
     * \param type The type of structure has that shape.
     * \param layer_index The index of the layer this structure is on.
     * \param z The Z coordinate of that layer.
     */
    void sendStructurePolygon(const Polygons& outline, const PrintFeatureType& type, const LayerIndex layer_index, const coord_t z) override;
};

}

#endif //VTK_OUTPUT
#endif //VTKCOMMUNICATION_H
