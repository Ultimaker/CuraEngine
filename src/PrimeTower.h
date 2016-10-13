#ifndef PRIME_TOWER_H
#define PRIME_TOWER_H

#include <vector>

#include "GCodePathConfig.h"
#include "MeshGroup.h"
#include "utils/polygon.h" // Polygons
#include "utils/polygonUtils.h"

namespace cura 
{

    
class SliceDataStorage;
class GCodePlanner;
class GCodeExport;

/*!
 * Class for everything to do with the prime tower:
 * - generating the areas
 * - checking up till which height the prime tower has to be printed
 * - generating the paths and adding them to the layer plan
 */
class PrimeTower
{
private:
    struct ExtrusionMoves
    {
        Polygons polygons;
        Polygons lines;
    };
    int extruder_count; //!< number of extruders
    std::vector<GCodePathConfig> config_per_extruder; //!< Path config for prime tower for each extruder

    bool is_hollow; //!< Whether the prime tower is hollow

    Point middle; //!< The middle of the prime tower

    Point post_wipe_point; //!< location to post-wipe the unused nozzle off on

    std::vector<ClosestPolygonPoint> pre_wipe_locations; //!< The differernt locations where to pre-wipe the active nozzle
    const unsigned int pre_wipe_location_skip = 8; //!< How big the steps are when stepping through \ref PrimeTower::wipe_locations
    const unsigned int number_of_pre_wipe_locations = 13; //!< The required size of \ref PrimeTower::wipe_locations
    // note that the above are two consecutive numbers in the fibonacci sequence
    int current_pre_wipe_location_idx; //!< Index into \ref PrimeTower::wipe_locations of where to pre-wipe the nozzle

public:
    Polygons ground_poly; //!< The outline of the prime tower to be used for each layer

    std::vector<std::vector<ExtrusionMoves>> patterns_per_extruder; //!< for each extruder a vector of patterns to alternate between, over the layers

    /*!
     * Initialize \ref PrimeTower::config_per_extruder with speed and line width settings.
     * 
     * \param meshgroup Where to retrieve the setttings for each extruder
     */
    void initConfigs(const MeshGroup* meshgroup);

    /*!
     * Complete the \ref PrimeTower::config_per_extruder by settings the layer height.
     * 
     * \param meshgroup Where to retrieve the setttings for each extruder
     * \param layer_thickness The current layer thickness
     */
    void setConfigs(const MeshGroup* meshgroup, const int layer_thickness);

    /*!
     * Generate the prime tower area to be used on each layer
     * 
     * Fills \ref PrimeTower::ground_poly and sets \ref PrimeTower::middle
     * 
     * \param storage Where to retrieve prime tower settings from
     */
    void generateGroundpoly(const SliceDataStorage& storage);

    /*!
     * Generate the area where the prime tower should be.
     * 
     * \param storage where to get settings from
     * \param total_layers The total number of layers 
     */
    void generatePaths(const SliceDataStorage& storage, unsigned int total_layers);

    PrimeTower(); //!< basic constructor

    /*!
     * Add path plans for the prime tower to the \p gcode_layer
     * 
     * \param storage where to get settings from; where to get the maximum height of the prime tower from
     * \param[in,out] gcode_layer Where to get the current extruder from; where to store the generated layer paths
     * \param layer_nr The layer for which to generate the prime tower paths
     * \param prev_extruder The previous extruder with which paths were planned; from which extruder a switch was made
     * \param new_extruder The switched to extruder with which the prime tower paths should be generated.
     */
    void addToGcode(const SliceDataStorage& storage, GCodePlanner& gcode_layer, const GCodeExport& gcode, const int layer_nr, const int prev_extruder, const int new_extruder);
private:
    /*!
     * Layer number of the last layer in which a prime tower has been printed per extruder train.  
     * 
     * This is recorded per extruder to account for a prime tower per extruder, instead of the mixed prime tower.
     */
    int last_prime_tower_poly_printed[MAX_EXTRUDERS];

    /*!
     * Find an approriate representation for the point representing the location before going to the prime tower
     * 
     * \warning This is not the actual position each time before the wipe tower
     * 
     * \param storage where to get settings from
     * \return that location
     */
    Point getLocationBeforePrimeTower(const SliceDataStorage& storage);

    /*!
     * \param storage where to get settings from
     * Depends on ground_poly being generated
     */
    void generateWipeLocations(const SliceDataStorage& storage);

    /*!
     * \see WipeTower::generatePaths
     * 
     * Generate the extrude paths for each extruder on even and odd layers
     * Fill the ground poly with dense infill.
     * 
     * \param storage where to get settings from
     * \param total_layers The total number of layers 
     */
    void generatePaths_denseInfill(const SliceDataStorage& storage);

    /*!
     * \see PrimeTower::addToGcode
     * 
     * Add path plans for the prime tower to the \p gcode_layer
     * 
     * \param storage where to get settings from; where to get the maximum height of the prime tower from
     * \param[in,out] gcode_layer Where to get the current extruder from; where to store the generated layer paths
     * \param layer_nr The layer for which to generate the prime tower paths
     * \param prev_extruder The previous extruder with which paths were planned; from which extruder a switch was made
     * \param new_extruder The switched to extruder with which the prime tower paths should be generated.
     */
    void addToGcode_denseInfill(const SliceDataStorage& storage, GCodePlanner& gcode_layer, const GCodeExport& gcode, const int layer_nr, const int prev_extruder, const int new_extruder);

    /*!
     * Plan the moves for wiping the current nozzles oozed material before starting to print the prime tower.
     * 
     * \param storage where to get settings from
     * \param[out] gcode_layer where to add the planned paths for wiping
     * \param extruder_nr The current extruder
     */
    void preWipe(const SliceDataStorage& storage, GCodePlanner& gcode_layer, const int extruder_nr);
};




}//namespace cura

#endif // PRIME_TOWER_H