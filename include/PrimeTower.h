// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PRIME_TOWER_H
#define PRIME_TOWER_H

#include <map>
#include <vector>

#include "ExtruderUse.h"
#include "settings/EnumSettings.h"
#include "settings/types/LayerIndex.h"
#include "utils/polygon.h" // Polygons
#include "utils/polygonUtils.h"

namespace cura
{

class SliceDataStorage;
class LayerPlan;

/*!
 * Class for everything to do with the prime tower:
 * - Generating the areas.
 * - Checking up untill which height the prime tower has to be printed.
 * - Generating the paths and adding them to the layer plan.
 */
class PrimeTower
{
private:
    using MovesByExtruder = std::vector<Polygons>;
    using MovesByLayer = std::vector<MovesByExtruder>;

    size_t extruder_count; //!< Number of extruders

    bool wipe_from_middle; //!< Whether to wipe on the inside of the hollow prime tower
    Point middle; //!< The middle of the prime tower

    Point post_wipe_point; //!< Location to post-wipe the unused nozzle off on

    std::vector<ClosestPolygonPoint> prime_tower_start_locations; //!< The differernt locations where to pre-wipe the active nozzle
    const unsigned int number_of_prime_tower_start_locations = 21; //!< The required size of \ref PrimeTower::wipe_locations

    MovesByExtruder prime_moves; //!< For each extruder, the moves to be processed for actual priming.
    std::map<size_t, std::map<size_t, Polygons>>
        sparse_pattern_per_extruders; //!< For each extruders combination, and for each actual extruder, the pattern to print on all layers where extruders are actually useless.
    MovesByLayer base_extra_moves; //!< For each layer and each extruder, the extra moves to be processed for better adhesion/strength

    Polygons outer_poly; //!< The outline of the outermost prime tower.
    std::vector<Polygons> outer_poly_base; //!< The outline of the layers having extra width for the base

public:
    bool enabled; //!< Whether the prime tower is enabled.
    bool would_have_actual_tower; //!< Whether there is an actual tower.
    bool multiple_extruders_on_first_layer; //!< Whether multiple extruders are allowed on the first layer of the prime tower (e.g. when a raft is there)

    /*
     * In which order, from outside to inside, will we be printing the prime
     * towers for maximum strength?
     *
     * This is the spatial order from outside to inside. This is NOT the actual
     * order in time in which they are printed.
     */
    std::vector<size_t> extruder_order;

    /*!
     * \brief Creates a prime tower instance that will determine where and how
     * the prime tower gets printed.
     *
     * \param storage A storage where it retrieves the prime tower settings.
     */
    PrimeTower();

    /*!
     * Check whether we actually use the prime tower.
     */
    void checkUsed(const SliceDataStorage& storage);

    /*!
     * Generate the prime tower area to be used on each layer
     *
     * Fills \ref PrimeTower::inner_poly and sets \ref PrimeTower::middle
     */
    void generateGroundpoly();

    /*!
     * Generate the area where the prime tower should be.
     */
    void generatePaths(const SliceDataStorage& storage);

    /*!
     * Add path plans for the prime tower to the \p gcode_layer
     *
     * \param storage where to get settings from; where to get the maximum height of the prime tower from
     * \param[in,out] gcode_layer Where to get the current extruder from; where to store the generated layer paths
     * \param required_extruder_prime the extruders which actually required to be primed at this layer
     * \param prev_extruder The previous extruder with which paths were planned; from which extruder a switch was made
     * \param new_extruder The switched to extruder with which the prime tower paths should be generated.
     */
    void addToGcode(
        const SliceDataStorage& storage,
        LayerPlan& gcode_layer,
        const std::vector<ExtruderUse>& required_extruder_prime,
        const size_t prev_extruder,
        const size_t new_extruder) const;

    /*!
     * \brief Subtract the prime tower from the support areas in storage.
     *
     * \param storage The storage where to find the support from which to
     * subtract a prime tower.
     */
    void subtractFromSupport(SliceDataStorage& storage);

    /*!
     * Get the outer polygon for the given layer, which may be the priming polygon only, or a larger polygon for layers with a base
     *
     * \param[in] layer_nr The index of the layer
     * \return The outer polygon for the prime tower at the given layer
     */
    const Polygons& getOuterPoly(const LayerIndex& layer_nr) const;

    /*!
     * Get the outer polygon for the very first layer, which may be the priming polygon only, or a larger polygon if there is a base
     */
    const Polygons& getGroundPoly() const;

private:
    /*!
     * \see PrimeTower::generatePaths
     *
     * Generate the extrude paths for each extruder on even and odd layers
     * Fill the ground poly with dense infill.
     * \param cumulative_insets [in, out] The insets added to each extruder to compute the radius of its ring
     */
    void generatePaths_denseInfill(std::vector<coord_t>& cumulative_insets);

    /*!
     * \see WipeTower::generatePaths
     *
     * \brief Generate the sparse extrude paths for each extruders combination
     * \param cumulative_insets The insets added to each extruder to compute the radius of its ring
     */
    void generatePaths_sparseInfill(const std::vector<coord_t>& cumulative_insets);

    /*!
     * \brief Generate the sparse extrude paths for an extruders combination
     *
     * \param first_extruder The index of the first extruder to be pseudo-primed
     * \param last_extruder The index of the last extruder to be pseudo-primed
     * \param rings_radii The external radii of each extruder ring, plus the internal radius of the internal ring
     * \param line_width The actual line width of the extruder
     * \param actual_extruder_nr The actual extruder to be used
     */
    Polygons generatePath_sparseInfill(
        const size_t first_extruder,
        const size_t last_extruder,
        const std::vector<coord_t>& rings_radii,
        const coord_t line_width,
        const size_t actual_extruder_nr);

    /*!
     * Generate start locations on the prime tower. The locations are evenly spread around the prime tower's perimeter.
     * The number of starting points is defined by "number_of_prime_tower_start_locations". The generated points will
     * be stored in "prime_tower_start_locations".
     */
    void generateStartLocations();

    /*!
     * \see PrimeTower::addToGcode
     *
     * Add path plans for the prime tower to the \p gcode_layer
     *
     * \param[in,out] gcode_layer Where to get the current extruder from. Where
     * to store the generated layer paths.
     * \param extruder The extruder we just switched to, with which the prime
     * tower paths should be drawn.
     */
    void addToGcode_denseInfill(LayerPlan& gcode_layer, const size_t extruder) const;

    void addToGcode_base(LayerPlan& gcode_layer, const size_t extruder_nr) const;

#warning TBD documentation
    void addToGcode_optimizedInfill(LayerPlan& gcode_layer, const std::vector<size_t>& extruders_to_prime, const size_t current_extruder) const;

    std::vector<size_t> findExtrudersSparseInfill(
        LayerPlan& gcode_layer,
        const std::vector<ExtruderUse>& required_extruder_prime,
        const size_t current_extruder,
        cura::PrimeTowerMethod method,
        const std::vector<size_t>& initial_list = {}) const;

    /*!
     * For an extruder switch that happens not on the first layer, the extruder needs to be primed on the prime tower.
     * This function picks a start location for this extruder on the prime tower's perimeter and travels there to avoid
     * starting at the location everytime which can result in z-seam blobs.
     */
    void gotoStartLocation(LayerPlan& gcode_layer, const int extruder) const;
};


} // namespace cura

#endif // PRIME_TOWER_H
