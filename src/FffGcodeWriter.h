//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GCODE_WRITER_H
#define GCODE_WRITER_H

#include <fstream>
#include <optional>

#include "FanSpeedLayerTime.h"
#include "gcodeExport.h"
#include "LayerPlanBuffer.h"
#include "settings/PathConfigStorage.h" //For the MeshPathConfigs subclass.
#include "utils/ExtrusionLine.h" //Processing variable-width paths.
#include "utils/NoCopy.h"

namespace cura 
{

class AngleDegrees;
class Polygons;
class SkinPart;
class SliceDataStorage;
class SliceMeshStorage;
class SliceLayer;
class SliceLayerPart;
class TimeKeeper;

/*!
 * Secondary stage in Fused Filament Fabrication processing: The generated polygons are used in the gcode generation.
 * Some polygons in the SliceDataStorage signify areas which are to be filled with parallel lines, 
 * while other polygons signify the contours which should be printed.
 * 
 * The main function of this class is FffGcodeWriter::writeGCode().
 */
class FffGcodeWriter : public NoCopy
{
    friend class Scene; // cause WireFrame2Gcode uses the member [gcode] (TODO)
    friend class FffProcessor; //Because FffProcessor exposes finalize (TODO)
private:
    coord_t max_object_height; //!< The maximal height of all previously sliced meshgroups, used to avoid collision when moving to the next meshgroup to print.

    /*
     * Buffer for all layer plans (of type LayerPlan)
     * 
     * The layer plans are buffered so that we can start heating up a nozzle several layers before it needs to be used.
     * Another reason is to perform Auto Temperature.
     */
    LayerPlanBuffer layer_plan_buffer; 

    /*!
     * The class holding the current state of the gcode being written.
     * 
     * It holds information such as the last written position etc.
     */
    GCodeExport gcode;

    /*!
     * The gcode file to write to when using CuraEngine as command line tool.
     */
    std::ofstream output_file;

    /*!
     * For each raft/filler layer, the extruders to be used in that layer in the order in which they are going to be used.
     * The first number is the first raft layer. Indexing is shifted compared to normal negative layer numbers for raft/filler layers.
     */
    std::vector<std::vector<size_t>> extruder_order_per_layer_negative_layers;

    std::vector<std::vector<size_t>> extruder_order_per_layer; //!< For each layer, the extruders to be used in that layer in the order in which they are going to be used

    std::vector<std::vector<size_t>> mesh_order_per_extruder; //!< For each extruder, the order of the meshes (first element is first mesh to be printed)

    /*!
     * For each extruder on which layer the prime will be planned,
     * or a large negative number if it's already planned outside of \ref FffGcodeWriter::processLayer
     * 
     * Depending on whether we need to prime on the first layer, or anywhere in the print,
     * the layer numbers are all zero (or less in case of raft)
     * or they are the first layer at which the extruder is needed
     */
    LayerIndex extruder_prime_layer_nr[MAX_EXTRUDERS];

    std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder; //!< The settings used relating to minimal layer time and fan speeds. Configured for each extruder.

public:
    /*
     * \brief Construct a g-code writer.
     *
     * This sets the initial state of the printer correctly in itself, so that
     * it's ready for writing.
     */
    FffGcodeWriter();

    /*!
     * Set the target to write gcode to: to a file.
     * 
     * Used when CuraEngine is used as command line tool.
     * 
     * \param filename The filename of the file to which to write the gcode.
     */
    bool setTargetFile(const char* filename);

    /*!
     * Set the target to write gcode to: an output stream.
     * 
     * Used when CuraEngine is NOT used as command line tool.
     * 
     * \param stream The stream to write gcode to.
     */
    void setTargetStream(std::ostream* stream);

    /*!
     * Get the total extruded volume for a specific extruder in mm^3
     * 
     * Retractions and unretractions don't contribute to this.
     * 
     * \param extruder_nr The extruder number for which to get the total netto extruded volume
     * \return total filament printed in mm^3
     */
    double getTotalFilamentUsed(int extruder_nr);

    /*!
     * Get the total estimated print time in seconds for each feature
     * 
     * \return total print time in seconds for each feature
     */
    std::vector<Duration> getTotalPrintTimePerFeature();

    /*!
     * Write all the gcode for the current meshgroup.
     * This is the primary function of this class.
     * 
     * \param[in] storage The data storage from which to get the polygons to print and the areas to fill.
     * \param timeKeeper The stop watch to see how long it takes for each of the stages in the slicing process.
     */
    void writeGCode(SliceDataStorage& storage, TimeKeeper& timeKeeper);

private:
    /*!
     * \brief Set the FffGcodeWriter::fan_speed_layer_time_settings by
     * retrieving all settings from the global/per-meshgroup settings.
     */
    void setConfigFanSpeedLayerTime();

    /*!
     * Create and set the SliceDataStorage::coasting_config for each extruder.
     * 
     * \param[out] storage The data storage to which to save the configuration
     */
    void setConfigCoasting(SliceDataStorage& storage);

    /*!
     * Set the retraction config globally, per extruder and per mesh.
     * 
     * \param[out] storage The data storage to which to save the configurations
     */
    void setConfigRetraction(SliceDataStorage& storage);

    /*!
     * Set the wipe config globally, per extruder.
     *
     * \param[out] storage The data storage to which to save the configurations
     */
    void setConfigWipe(SliceDataStorage& storage);

    /*!
     * Get the extruder with which to start the print.
     * 
     * Generally this is the adhesion_extruder_nr, but in case the platform adhesion type is none,
     * the extruder with lowest number which is used on the first layer is used as initial extruder.
     * 
     * \param[in] storage where to get settings from.
     */
    unsigned int getStartExtruder(const SliceDataStorage& storage);

    /*!
     * Set the infill angles and skin angles in the SliceDataStorage.
     * 
     * These lists of angles are cycled through to get the infill angle of a specific layer.
     * 
     * \param mesh The mesh for which to determine the infill and skin angles.
     */
    void setInfillAndSkinAngles(SliceMeshStorage& mesh);

    /*!
     * Set the support and interface infill angles in the SliceDataStorage.
     *
     * Default angles depend on which pattern it's using and in certain patterns it
     * alternates between layers.
     *
     * These lists of angles are cycled through to get the support infill angle of a specific layer.
     *
     * \param storage The storage for which to determine the support infill angles.
     */
    void setSupportAngles(SliceDataStorage& storage);

    /*!
    * Set temperatures for the initial layer. Called by 'processStartingCode' and whenever a new object is started at layer 0.
    *
    * \param[in] storage where the slice data is stored.
    * \param[in] start_extruder_nr The extruder with which to start the print.
    */
    void processInitialLayerTemperature(const SliceDataStorage& storage, const size_t start_extruder_nr);

    /*!
     * Set temperatures and perform initial priming.
     * 
     * Write a stub header if CuraEngine is in command line tool mode. (Cause writing the header afterwards would entail moving all gcode down.)
     * 
     * \param[in] storage where the slice data is stored.
     * \param[in] start_extruder_nr The extruder with which to start the print.
     */
    void processStartingCode(const SliceDataStorage& storage, const size_t start_extruder_nr);

    /*!
     * Move up and over the already printed meshgroups to print the next meshgroup.
     * 
     * \param[in] storage where the slice data is stored.
     */
    void processNextMeshGroupCode(const SliceDataStorage& storage);
    
    /*!
     * Add raft layer plans onto the FffGcodeWriter::layer_plan_buffer
     * 
     * \param[in,out] storage where the slice data is stored.
     */
    void processRaft(const SliceDataStorage& storage);

    /*!
     * Convert the polygon data of a layer into a layer plan on the FffGcodeWriter::layer_plan_buffer
     * 
     * In case of negative layer numbers, create layers only containing the data from
     * the helper parts (support etc) to fill up the gap between the raft and the model.
     * 
     * \param[in] storage where the slice data is stored.
     * \param layer_nr The index of the layer to write the gcode of.
     * \param total_layers The total number of layers.
     * \return The layer plans
     */
    LayerPlan& processLayer(const SliceDataStorage& storage, LayerIndex layer_nr, const size_t total_layers) const;

    /*!
     * This function checks whether prime blob should happen for any extruder on the first layer.
     * Priming will always happen, but the actual priming may or may not include a prime blob.
     *
     * Technically, this function checks whether any extruder needs to be primed (with a prime blob)
     * separately just before they are used.
     * 
     * \return whether any extruder need to be primed separately just before they are used
     */
    bool getExtruderNeedPrimeBlobDuringFirstLayer(const SliceDataStorage& storage, const size_t extruder_nr) const;

    /*!
     * Plan priming of all used extruders which haven't been primed yet
     * \param[in] storage where the slice data is stored.
     * \param layer_plan The initial planning of the g-code of the layer.
     */
    void ensureAllExtrudersArePrimed(const SliceDataStorage& storage, LayerPlan& layer_plan) const;

    /*!
     * Add the skirt or the brim to the layer plan \p gcodeLayer if it hasn't already been added yet.
     * 
     * This function should be called for only one layer;
     * calling it for multiple layers results in the skirt/brim being printed on multiple layers.
     * 
     * \param storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the g-code of the layer.
     * \param extruder_nr The extruder train for which to process the skirt or
     * brim.
     */
    void processSkirtBrim(const SliceDataStorage& storage, LayerPlan& gcodeLayer, unsigned int extruder_nr) const;

    /*!
     * Adds the ooze shield to the layer plan \p gcodeLayer.
     * 
     * \param[in] storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     */
    void processOozeShield(const SliceDataStorage& storage, LayerPlan& gcodeLayer) const;
    
    /*!
     * Adds the draft protection screen to the layer plan \p gcodeLayer.
     * 
     * \param[in] storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     */
    void processDraftShield(const SliceDataStorage& storage, LayerPlan& gcodeLayer) const;

    /*!
     * Calculate in which order to plan the extruders for each layer
     * Store the order of extruders for each layer in extruder_order_per_layer for normal layers
     * and the order of extruders for raft/filler layers in extruder_order_per_layer_negative_layers.
     * 
     * Only extruders which are (most probably) going to be used are planned
     * 
     * \note At the planning stage we only have information on areas, not how those are filled.
     * If an area is too small to be filled with anything it will still get specified as being used with the extruder for that area.
     * 
     * Computes \ref FffGcodeWriter::extruder_order_per_layer and \ref FffGcodeWriter::extruder_order_per_layer_negative_layers
     * 
     * \param[in] storage where the slice data is stored.
     */
    void calculateExtruderOrderPerLayer(const SliceDataStorage& storage);

    /*!
     * Calculate on which layer we should be priming for each extruder.
     *
     * The extruders are primed on the lowest layer at which they are used.
     * \param storage Slice data storage containing information on which layers
     * each extruder is used.
     */
    void calculatePrimeLayerPerExtruder(const SliceDataStorage& storage);

    /*!
     * Gets a list of extruders that are used on the given layer, but excluding the given starting extruder.
     * When it's on the first layer, the prime blob will also be taken into account.
     * 
     * \note At the planning stage we only have information on areas, not how those are filled.
     * If an area is too small to be filled with anything it will still get specified as being used with the extruder for that area.
     * 
     * \param[in] storage where the slice data is stored.
     * \param current_extruder The current extruder with which we last printed
     * \return The order of extruders for a layer beginning with \p current_extruder
     */
    std::vector<size_t> getUsedExtrudersOnLayerExcludingStartingExtruder(const SliceDataStorage& storage, const size_t start_extruder, const LayerIndex& layer_nr) const;

    /*!
     * Calculate in which order to plan the meshes of a specific extruder
     * Each mesh which has some feature printed with the extruder is included in this order.
     * One mesh can occur in the mesh order of multiple extruders.
     * 
     * \param[in] storage where the slice data is stored.
     * \param extruder_nr The extruder for which to determine the order
     * \return A vector of mesh indices ordered on print order for that extruder.
     */
    std::vector<size_t> calculateMeshOrder(const SliceDataStorage& storage, const size_t extruder_nr) const;

    /*!
     * Add a single layer from a single mesh-volume to the layer plan \p gcodeLayer in mesh surface mode.
     * 
     * \param[in] storage where the slice data is stored.
     * \param mesh The mesh to add to the layer plan \p gcodeLayer.
     * \param mesh_config the line config with which to print a print feature
     * \param gcodeLayer The initial planning of the gcode of the layer.
     */
    void addMeshLayerToGCode_meshSurfaceMode(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcodeLayer) const;
    
    /*!
     * Add the open polylines from a single layer from a single mesh-volume to the layer plan \p gcodeLayer for mesh the surface modes.
     * 
     * \param[in] storage where the slice data is stored.
     * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
     * \param mesh_config the line config with which to print a print feature
     * \param gcodeLayer The initial planning of the gcode of the layer.
     */
    void addMeshOpenPolyLinesToGCode(const SliceMeshStorage& mesh, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcode_layer) const;
    
    /*!
     * Add all features of a given extruder from a single layer from a single mesh-volume to the layer plan \p gcode_layer.
     * 
     * This adds all features (e.g. walls, skin etc.) of this \p mesh to the gcode which are printed using \p extruder_nr
     * 
     * \param[in] storage where the slice data is stored.
     * \param mesh The mesh to add to the layer plan \p gcode_layer.
     * \param extruder_nr The extruder for which to print all features of the mesh which should be printed with this extruder
     * \param mesh_config the line config with which to print a print feature
     * \param gcode_layer The initial planning of the gcode of the layer.
     */
    void addMeshLayerToGCode(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcode_layer) const;

    /*!
     * Add all features of the given extruder from a single part from a given layer of a mesh-volume to the layer plan \p gcode_layer.
     * This only adds the features which are printed with \p extruder_nr.
     * 
     * \param[in] storage where the slice data is stored.
     * \param storage Storage to get global settings from.
     * \param mesh The mesh to add to the layer plan \p gcode_layer.
     * \param extruder_nr The extruder for which to print all features of the mesh which should be printed with this extruder
     * \param mesh_config the line config with which to print a print feature
     * \param part The part to add
     * \param gcode_layer The initial planning of the gcode of the layer.
     */
    void addMeshPartToGCode(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, LayerPlan& gcode_layer) const;

    /*!
     * \brief Add infill for a given part in a layer plan.
     *
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
     * \param extruder_nr The extruder for which to print all features of the
     * mesh which should be printed with this extruder.
     * \param mesh_config the line config with which to print a print feature.
     * \param part The part for which to create gcode.
     * \return Whether this function added anything to the layer plan.
     */
    bool processInfill(const SliceDataStorage& storage, LayerPlan& gcodeLayer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const;

    /*!
     * \brief Add thicker (multiple layers) sparse infill for a given part in a
     * layer plan.
     * 
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
     * \param extruder_nr The extruder for which to print all features of the
     * mesh which should be printed with this extruder.
     * \param mesh_config The line config with which to print a print feature.
     * \param part The part for which to create gcode.
     * \return Whether this function added anything to the layer plan.
     */
    bool processMultiLayerInfill(const SliceDataStorage& storage, LayerPlan& gcodeLayer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const;

    /*!
     * \brief Add normal sparse infill for a given part in a layer.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
     * \param extruder_nr The extruder for which to print all features of the
     * mesh which should be printed with this extruder
     * \param mesh_config The line config with which to print a print feature.
     * \param part The part for which to create gcode.
     * \return Whether this function added anything to the layer plan.
     */
    bool processSingleLayerInfill(const SliceDataStorage& storage, LayerPlan& gcodeLayer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const;

    /*!
     * Generate the insets for the walls of a given layer part.
     * \param[in] storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
     * \param extruder_nr The extruder for which to print all features of the mesh which should be printed with this extruder
     * \param mesh_config the line config with which to print a print feature
     * \param part The part for which to create gcode
     * \return Whether this function added anything to the layer plan
     */
    bool processInsets(const SliceDataStorage& storage, LayerPlan& gcodeLayer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const;

    /*!
     * Generate the a spiralized wall for a given layer part.
     * \param[in] storage where the slice data is stored.
     * \param[out] gcodeLayer The initial planning of the gcode of the layer.
     * \param mesh_config the line config with which to print a print feature
     * \param part The part for which to create gcode
     * \param mesh The mesh for which to add to the layer plan \p gcodeLayer.
     */
    void processSpiralizedWall(const SliceDataStorage& storage, LayerPlan& gcode_layer, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, const SliceMeshStorage& mesh) const;

    /*!
     * Add the gcode of the top/bottom skin of the given part and of the perimeter gaps.
     *
     * \param[in] storage where the slice data is stored.
     * \param gcode_layer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcode_layer.
     * \param extruder_nr The extruder for which to print all features of the mesh which should be printed with this extruder
     * \param mesh_config the line config with which to print a print feature
     * \param part The part for which to create gcode
     * \return Whether this function added anything to the layer plan
     */
    bool processSkin(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const;

    /*!
     * Add the gcode of the top/bottom skin of the given skin part and of the perimeter gaps.
     * 
     * Perimeter gaps are handled for the current extruder for the following features if they are printed with this extruder.
     * - skin outlines
     * - roofing (if concentric)
     * - top/bottom (if concentric)
     * They are all printed at the end of printing the skin part features which are printed with this extruder.
     * 
     * Note that the normal perimeter gaps are printed with the outer wall extruder,
     * while newly generated perimeter gaps
     * are printed with the extruder with which the feature was printed which generated the gaps.
     * 
     * \param[in] storage where the slice data is stored.
     * \param gcode_layer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcode_layer.
     * \param extruder_nr The extruder for which to print all features of the mesh which should be printed with this extruder
     * \param mesh_config the line config with which to print a print feature
     * \param skin_part The skin part for which to create gcode
     * \return Whether this function added anything to the layer plan
     */
    bool processSkinPart(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part) const;

    /*!
     * Add the roofing which is the area inside the innermost skin inset which has air 'directly' above
     *
     * \param[in] storage where the slice data is stored.
     * \param gcode_layer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcode_layer.
     * \param extruder_nr The extruder for which to print all features of the mesh which should be printed with this extruder
     * \param mesh_config the line config with which to print a print feature
     * \param skin_part The skin part for which to create gcode
     * \param[out] added_something Whether this function added anything to the layer plan
     */
    void processRoofing(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part, bool& added_something) const;

    /*!
     * Add the normal skinfill which is the area inside the innermost skin inset
     * which doesn't have air directly above it if we're printing roofing
     *
     * \param[in] storage where the slice data is stored.
     * \param gcode_layer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcode_layer.
     * \param extruder_nr The extruder for which to print all features of the mesh which should be printed with this extruder
     * \param mesh_config the line config with which to print a print feature
     * \param skin_part The skin part for which to create gcode
     * \param[out] added_something Whether this function added anything to the layer plan
     */
    void processTopBottom(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part, bool& added_something) const;

    /*!
     * Process a dense skin feature like roofing or top/bottom
     * 
     * \param[in] storage where the slice data is stored.
     * \param gcode_layer The initial planning of the gcode of the layer.
     * \param mesh The mesh for which to add to the layer plan \p gcode_layer.
     * \param mesh_config The mesh-config for which to add to the layer plan \p gcode_layer.
     * \param extruder_nr The extruder for which to print all features of the mesh which should be printed with this extruder
     * \param area The area to fill
     * \param config the line config with which to print the print feature
     * \param pattern the pattern with which to fill the print feature
     * \param skin_angle the angle to use for linear infill types
     * \param skin_overlap The amount by which to expand the \p area
     * \param skin density Sets the density of the the skin lines by adjusting the distance between them (normal skin is 1.0)
     * \param monotonic Whether to order lines monotonically (``true``) or to
     * minimise travel moves (``false``).
     * \param[out] added_something Whether this function added anything to the layer plan
     * \param fan_speed fan speed override for this skin area
     */
    void processSkinPrintFeature(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const PathConfigStorage::MeshPathConfigs& mesh_config, const size_t extruder_nr, const Polygons& area, const GCodePathConfig& config, EFillMethod pattern, const AngleDegrees skin_angle, const coord_t skin_overlap, const Ratio skin_density, const bool monotonic, bool& added_something, double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT) const;

    /*!
     *  see if we can avoid printing a lines or zig zag style skin part in multiple segments by moving to
     *  a start point that would increase the chance that the skin will be printed in a single segment.
     *  Obviously, if the skin part contains holes then it will have to be printed in multiple segments anyway but
     *  doing this may still produce fewer skin seams or move a seam that would be across the middle of the part
     *  to a less noticeable position
     *
     * So, instead of this \/         We get this \/
     *                +------+               +------+
     *                |2///#1|               |1/////|
     *                |///#//|               |//////|
     *                |//#///|               |//////|
     *                |/#////|               |//////|
     *                |#/////|               |//////|
     *                +------+               +------+
     *     1, 2 = start locations of skin segments
     *     # = seam
     * 
     * \param filling_part The part which we are going to fill with a linear filling type
     * \param filling_angle The angle of the filling lines
     * \param last_position The position the print head is in before going to fill the part
     * \return The location near where to start filling the part
     */
    std::optional<Point> getSeamAvoidingLocation(const Polygons& filling_part, int filling_angle, Point last_position) const;

    /*!
     * Add the g-code for ironing the top surface.
     *
     * This produces additional low-extrusion moves that cover the top surface,
     * in order to smooth the surface more.
     *
     * \param mesh The settings storage to get the ironing settings and skin
     * angles from.
     * \param layer The layer to process the ironing for.
     * \param line_config The configuration of the lines to draw the ironing
     * with.
     * \param[out] gcode_layer The output layer to put the resulting paths in.
     * \return Whether this function added anything to the layer plan.
     */
    bool processIroning(const SliceMeshStorage& mesh, const SliceLayer& part, const GCodePathConfig& line_config, LayerPlan& gcode_layer) const;

    /*!
     * Add the support to the layer plan \p gcodeLayer of the current layer for all support parts with the given \p extruder_nr.
     * \param[in] storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \return whether any support was added to the layer plan
     */
    bool addSupportToGCode(const SliceDataStorage& storage, LayerPlan& gcodeLayer, const size_t extruder_nr) const;

    /*!
     * Add the support lines/walls to the layer plan \p gcodeLayer of the current layer.
     * \param[in] storage where the slice data is stored.
     * \param gcode_layer The initial planning of the gcode of the layer.
     * \return whether any support infill was added to the layer plan
     */
    bool processSupportInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer) const;

    /*!
     * Add the support roofs to the layer plan \p gcodeLayer of the current
     * layer.
     *
     * \param[in] storage Where the slice data is stored.
     * \param gcodeLayer The initial planning of the g-code of the layer.
     * \return Whether any support skin was added to the layer plan.
     */
    bool addSupportRoofsToGCode(const SliceDataStorage& storage, LayerPlan& gcodeLayer) const;

    /*!
     * Add the support bottoms to the layer plan \p gcodeLayer of the current
     * layer.
     *
     * \param[in] storage Where the slice data is stored.
     * \param gcodeLayer The initial planning of the g-code of the layer.
     * \return Whether any support skin was added to the layer plan.
     */
    bool addSupportBottomsToGCode(const SliceDataStorage& storage, LayerPlan& gcodeLayer) const;

public:
    /*!
     * Change to a new extruder, and add the prime tower instructions if the new extruder is different from the last.
     * 
     * On layer 0 this function adds the skirt for the nozzle it switches to, instead of the prime tower.
     * 
     * \param[in] storage where the slice data is stored.
     * \param gcode_layer The initial planning of the gcode of the layer.
     * \param extruder_nr The extruder to switch to.
     */
    void setExtruder_addPrime(const SliceDataStorage& storage, LayerPlan& gcode_layer, const size_t extruder_nr) const;

private:
    /*!
     * Add the prime tower gcode for the current layer.
     * \param[in] storage where the slice data is stored.
     * \param gcodeLayer The initial planning of the gcode of the layer.
     * \param prev_extruder The current extruder with which we last printed.
     */
    void addPrimeTower(const SliceDataStorage& storage, LayerPlan& gcodeLayer, const size_t prev_extruder) const;
    
    /*!
     * Add the end gcode and set all temperatures to zero.
     */
    void finalize();

    /*!
     * Calculate for each layer the index of the vertex that is considered to be the seam
     * \param storage where the slice data is stored.
     * \param total_layers The total number of layers
     */
    void findLayerSeamsForSpiralize(SliceDataStorage& storage, size_t total_layers);

    /*!
     * Calculate the index of the vertex that is considered to be the seam for the given layer
     * \param storage where the slice data is stored.
     * \param mesh the mesh containing the layer of interest
     * \param layer_nr layer number of the layer whose seam verted index is required
     * \param last_layer_nr layer number of the previous layer
     * \return layer seam vertex index
     */
    unsigned int findSpiralizedLayerSeamVertexIndex(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const int layer_nr, const int last_layer_nr);

    /*!
     * Partition the Infill regions by the skin at N layers above.
     *
     * When skin edge support layers is set this function will check N layers above the infill layer to see if there is
     * skin above. If this skin needs to be supported by a wall it will return true else it returns false. The Infill
     * outline of the Sparse density layer is partitioned into two polygons, either below the skin regions or outside
     * of the skin region.
     *
     * \param infill_below_skin [out] Polygons with infill below the skin
     * \param infill_not_below_skin [out] Polygons with infill outside of skin regions above
     * \param gcode_layer The initial planning of the gcode of the layer
     * \param mesh the mesh containing the layer of interest
     * \param part \param part The part for which to create gcode
     * \param infill_line_width line width of the infill
     * \return true if there needs to be a skin edge support wall in this layer, otherwise false
     */
    static bool partitionInfillBySkinAbove(Polygons& infill_below_skin, Polygons& infill_not_below_skin, const LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const SliceLayerPart& part, coord_t infill_line_width) ;
};

}//namespace cura

#endif // GCODE_WRITER_H
