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

    void initConfigs(MeshGroup* meshgroup, std::vector<RetractionConfig>& retraction_config_per_extruder);
    void setConfigs(MeshGroup* configs, int layer_thickness);

    void generateGroundpoly(const SliceDataStorage& storage);

    std::vector<std::vector<Polygons>> patterns_per_extruder; //!< for each extruder a vector of patterns to alternate between, over the layers

    /*!
     * Generate the area where the prime tower should be.
     * 
     * \param storage where to get settings from
     * \param total_layers The total number of layers 
     */
    void generatePaths(const SliceDataStorage& storage, unsigned int total_layers);

    void computePrimeTowerMax(SliceDataStorage& storage);

    PrimeTower();

    void addToGcode(const SliceDataStorage& storage, GCodePlanner& gcodeLayer, GCodeExport& gcode, int layer_nr, int prev_extruder, bool prime_tower_dir_outward, bool wipe, int* last_prime_tower_poly_printed);
private:
    /*!
     * \param storage where to get settings from
     * Depends on ground_poly being generated
     */
    void generateWipeLocations(const SliceDataStorage& storage);

    void generatePaths_denseInfill(const SliceDataStorage& storage);

    void addToGcode_denseInfill(const SliceDataStorage& storage, GCodePlanner& gcodeLayer, GCodeExport& gcode, int layer_nr, int prev_extruder, bool prime_tower_dir_outward, bool wipe, int* last_prime_tower_poly_printed);

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