//Copyright (c) 2016 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

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
class LayerPlan;
class GCodeExport;

/*!
 * Class for everything to do with the prime tower:
 * - Generating the areas.
 * - Checking up untill which height the prime tower has to be printed.
 * - Generating the paths and adding them to the layer plan.
 */
class PrimeTower
{
private:
    struct ExtrusionMoves
    {
        Polygons polygons;
        Polygons lines;
    };
    int extruder_count; //!< Number of extruders

    bool is_hollow; //!< Whether the prime tower is hollow

    bool wipe_from_middle; //!< Whether to wipe on the inside of the hollow prime tower
    Point middle; //!< The middle of the prime tower

    Point post_wipe_point; //!< Location to post-wipe the unused nozzle off on

    std::vector<ClosestPolygonPoint> pre_wipe_locations; //!< The differernt locations where to pre-wipe the active nozzle
    const unsigned int pre_wipe_location_skip = 13; //!< How big the steps are when stepping through \ref PrimeTower::wipe_locations
    const unsigned int number_of_pre_wipe_locations = 21; //!< The required size of \ref PrimeTower::wipe_locations
    // note that the above are two consecutive numbers in the Fibonacci sequence

    std::vector<std::vector<ExtrusionMoves>> patterns_per_extruder; //!< For each extruder a vector of patterns to alternate between, over the layers
    std::vector<ExtrusionMoves> pattern_per_extruder_layer0; //!< For each extruder the pattern to print on the first layer

public:
    bool enabled; //!< Whether the prime tower is enabled.
    Polygons ground_poly; //!< The outline of the prime tower to be used for each layer

    /*!
     * \brief Creates a prime tower instance that will determine where and how
     * the prime tower gets printed.
     *
     * \param storage A storage where it retrieves the prime tower settings.
     */
    PrimeTower(const SliceDataStorage& storage);

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
     */
    void generatePaths(const SliceDataStorage& storage);

    /*!
     * Add path plans for the prime tower to the \p gcode_layer
     * 
     * \param storage where to get settings from; where to get the maximum height of the prime tower from
     * \param[in,out] gcode_layer Where to get the current extruder from; where to store the generated layer paths
     * \param prev_extruder The previous extruder with which paths were planned; from which extruder a switch was made
     * \param new_extruder The switched to extruder with which the prime tower paths should be generated.
     */
    void addToGcode(const SliceDataStorage& storage, LayerPlan& gcode_layer, const GCodeExport& gcode, const int prev_extruder, const int new_extruder) const;

    /*!
     * \brief Subtract the prime tower from the support areas in storage.
     *
     * \param storage The storage where to find the support from which to
     * subtract a prime tower.
     */
    void subtractFromSupport(SliceDataStorage& storage);

private:

    /*!
     * Find an approriate representation for the point representing the location before going to the prime tower
     * 
     * \warning This is not the actual position each time before the wipe tower
     * 
     * \param storage where to get settings from
     * \return that location
     */
    Point getLocationBeforePrimeTower(const SliceDataStorage& storage) const;

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
     */
    void generatePaths_denseInfill(const SliceDataStorage& storage);

    /*!
     * \see PrimeTower::addToGcode
     *
     * Add path plans for the prime tower to the \p gcode_layer
     *
     * \param[in,out] gcode_layer Where to get the current extruder from; where to store the generated layer paths
     * \param extruder The extruder we just switched to, with which the prime
     * tower paths should be drawn.
     */
    void addToGcode_denseInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer, const int extruder) const;

    /*!
     * Plan the moves for wiping and purging (if enabled) the current nozzles oozed material before starting
     * to print the prime tower.
     * This generates wipe moves from inside the tower towards the outside if the tower is hollow. Otherwise
     * it wipes from outside to inside.
     * 
     * \param storage where to get settings from
     * \param[out] gcode_layer where to add the planned paths for wiping
     * \param extruder_nr The current extruder
     */
    void preWipeAndPurge(const SliceDataStorage& storage, LayerPlan& gcode_layer, const int extruder_nr) const;

    /*!
     * Plan a purge move using the prime tower.
     *
     * The purge move starts from the center of the prime tower and moves outwards towards the prime tower.
     * This is an extrusion move so the nozzle will extrude a certain amount of material during the move,
     * and this is called "purge". This feature can be enabled when prime tower wipe is enabled.
     *
     * \param gcode_layer the layer plan to used
     * \param extruder_nr the extruder number that will be purged
     * \param train the extruder train of the extruder what will be purged
     * \param start_pos the start position of the purge move
     * \param end_pos the end position of the purge move
     * \param purge_volume the purge volume in mm^3
     */
    void addPurgeMove(LayerPlan& gcode_layer, int extruder_nr, const ExtruderTrain *train, const Point& start_pos, const Point& end_pos, double purge_volume) const;
};




}//namespace cura

#endif // PRIME_TOWER_H
