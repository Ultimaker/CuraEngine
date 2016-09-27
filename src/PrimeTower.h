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

typedef std::vector<IntPoint> PolyLine;

/*!
 * Class for 
 */
class PrimeTower
{
private:
    int extruder_count; //!< number of extruders
    std::vector<GCodePathConfig> config_per_extruder; //!< Path config for prime tower for each extruder

    Point post_wipe_point; //!< location to post-wipe the unused nozzle off on

    std::vector<PolyLine> extruder_paths; //!< Precompiled so that we don't need to generate the paths each layer over again

    std::vector<ClosestPolygonPoint> pre_wipe_locations; //!< The differernt locations where to pre-wipe the active nozzle
    const unsigned int pre_wipe_location_skip = 8; //!< How big the steps are when stepping through \ref PrimeTower::wipe_locations
    const unsigned int number_of_pre_wipe_locations = 13; //!< The required size of \ref PrimeTower::wipe_locations
    // note that the above are two consecutive numbers in the fibonacci sequence
    int current_pre_wipe_location_idx; //!< Index into \ref PrimeTower::wipe_locations of where to pre-wipe the nozzle

public:
    Polygons ground_poly; //!< The outline of the prime tower to be used for each layer

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
     * \param storage Where to retrieve prime tower settings from
     */
    void generateGroundpoly(const SliceDataStorage& storage);

    std::vector<std::vector<Polygons>> patterns_per_extruder; //!< for each extruder a vector of patterns to alternate between, over the layers

    /*!
     * Generate the area where the prime tower should be.
     * 
     * \param storage where to get settings from
     * \param total_layers The total number of layers 
     */
    void generatePaths(const SliceDataStorage& storage, unsigned int total_layers);

    /*!
     * Compute the maximum layer at which a layer switch will occur and store the result in \ref SliceDataStorage::max_object_height_second_to_last_extruder
     * 
     * \param[in,out] storage Where to retrieve area data and extruder settings for those areas; where to store the max_object_height_second_to_last_extruder
     */
    void computePrimeTowerMax(SliceDataStorage& storage);

    PrimeTower(); //!< basic constructor

    /*!
     * Add path plans for the prime tower to the \p gcode_layer
     * 
     * \param storage where to get settings from; where to get the maximum height of the prime tower from
     * \param[in,out] gcode_layer Where to get the current extruder from; where to store the generated layer paths
     * \param layer_nr The layer for which to generate the prime tower paths
     * \param prev_extruder The previous extruder with which paths were planned; from which extruder a switch was made
     * \param wipe Whether to wipe of the (not previous, but) current nozzle on the wipe tower (only occurs if previous extruder is different fromt he current one)
     * \param[in,out] last_prime_tower_poly_printed At which layer a prime tower was (already) printed. (TODO: move inside this class)
     */
    void addToGcode(const SliceDataStorage& storage, GCodePlanner& gcode_layer, const GCodeExport& gcode, const int layer_nr, const int prev_extruder, bool wipe, int* last_prime_tower_poly_printed);
private:
    /*!
     * \param storage where to get settings from
     * Depends on ground_poly being generated
     */
    void generateWipeLocations(const SliceDataStorage& storage);

    /*!
     * \see WipeTower::generatePaths
     * 
     * Generate the area where the prime tower should be.
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
     * \param[in,out] last_prime_tower_poly_printed At which layer a prime tower was (already) printed. (TODO: move inside this class)
     */
    void addToGcode_denseInfill(const SliceDataStorage& storage, GCodePlanner& gcode_layer, const GCodeExport& gcode, const int layer_nr, const int prev_extruder, int* last_prime_tower_poly_printed);

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