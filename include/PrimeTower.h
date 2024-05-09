// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PRIME_TOWER_H
#define PRIME_TOWER_H

#include <map>
#include <vector>

#include "ExtruderUse.h"
#include "geometry/Polygon.h"
#include "settings/EnumSettings.h"
#include "settings/types/LayerIndex.h"
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
    using MovesByExtruder = std::map<size_t, Shape>;
    using MovesByLayer = std::map<size_t, std::vector<Shape>>;

    size_t extruder_count_; //!< Number of extruders

    bool wipe_from_middle_; //!< Whether to wipe on the inside of the hollow prime tower
    Point2LL middle_; //!< The middle of the prime tower

    Point2LL post_wipe_point_; //!< Location to post-wipe the unused nozzle off on

    std::vector<ClosestPointPolygon> prime_tower_start_locations_; //!< The differernt locations where to pre-wipe the active nozzle
    const unsigned int number_of_prime_tower_start_locations_ = 21; //!< The required size of \ref PrimeTower::wipe_locations

    MovesByExtruder prime_moves_; //!< For each extruder, the moves to be processed for actual priming.

    /*
     *  The first index is a bitmask representing an extruder combination, e.g. 0x05 for extruders 1+3.
     *  The second index is the used extruder index, e.g. 1
     *  The polygons represent the sparse pattern to be printed when all the given extruders are unused for this layer
     *  and the given extruder is currently in use
     */
    std::map<size_t, std::map<size_t, Shape>> sparse_pattern_per_extruders_;

    MovesByLayer base_extra_moves_; //!< For each layer and each extruder, the extra moves to be processed for better adhesion/strength
    MovesByExtruder inset_extra_moves_; //!< For each extruder, the extra inset moves to be processed for better adhesion on initial layer

    Shape outer_poly_; //!< The outline of the outermost prime tower.
    std::vector<Shape> outer_poly_base_; //!< The outline of the layers having extra width for the base

public:
    bool enabled_; //!< Whether the prime tower is enabled.

    /*
     * In which order, from outside to inside, will we be printing the prime
     * towers for maximum strength?
     *
     * This is the spatial order from outside to inside. This is NOT the actual
     * order in time in which they are printed.
     */
    std::vector<size_t> extruder_order_;

    /*!
     * \brief Creates a prime tower instance that will determine where and how
     * the prime tower gets printed.
     *
     * \param storage A storage where it retrieves the prime tower settings.
     */
    PrimeTower();

    void initializeExtruders(const std::vector<bool>& used_extruders);

    /*!
     * Check whether we actually use the prime tower.
     */
    void checkUsed();

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
     * \param prev_extruder_nr The previous extruder with which paths were planned; from which extruder a switch was made
     * \param new_extruder_nr The switched to extruder with which the prime tower paths should be generated.
     */
    void addToGcode(
        const SliceDataStorage& storage,
        LayerPlan& gcode_layer,
        const std::vector<ExtruderUse>& required_extruder_prime,
        const size_t prev_extruder_nr,
        const size_t new_extruder_nr) const;

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
    const Shape& getOuterPoly(const LayerIndex& layer_nr) const;

    /*!
     * Get the outer polygon for the very first layer, which may be the priming polygon only, or a larger polygon if there is a base
     */
    const Shape& getGroundPoly() const;

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
     * \param first_extruder_nr The index of the first extruder to be pseudo-primed
     * \param last_extruder_nr The index of the last extruder to be pseudo-primed
     * \param rings_radii The external radii of each extruder ring, plus the internal radius of the internal ring
     * \param line_width The actual line width of the extruder
     * \param actual_extruder_nr The number of the actual extruder to be used
     */
    Shape generatePath_sparseInfill(
        const size_t first_extruder_idx,
        const size_t last_extruder_idx,
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

    /*!
     * \brief Add path plans for the prime tower extra outer rings to make the stronger base
     * \param gcode_layer The gcode export to add the paths plans to
     * \param extruder_nr The current extruder number
     * \return True if something has actually been added, according to the extruder number
     *         and current layer.
     */
    bool addToGcode_base(LayerPlan& gcode_layer, const size_t extruder_nr) const;

    /*!
     * \brief Add path plans for the prime tower extra inner rings to increase bed adhesion
     * \param gcode_layer The gcode export to add the paths plans to
     * \param extruder_nr The current extruder number
     * \return True if something has actually been added, according to the extruder number
     *         and current layer.
     */
    bool addToGcode_inset(LayerPlan& gcode_layer, const size_t extruder_nr) const;

    /*!
     * \brief Add path plans in the case an extruder is not to be actually primed, but we still
     *        want to print something to make the prime tower consistent.
     * \param gcode_layer The gcode export to add the paths plans to
     * \param extruders_to_prime_idx The indexes of the extra extruders which also don't require being primed on this layer
     * \param current_extruder_nr The extruder currently being used
     */
    void addToGcode_sparseInfill(LayerPlan& gcode_layer, const std::vector<size_t>& extruders_to_prime_idx, const size_t current_extruder_nr) const;

    /*!
     * \brief Find the list of extruders that don't actually need to be primed during this layer, and for which
     *        we want to print only the sparse infill to keep the prime tower consistent.
     * \param gcode_layer The current gcode export
     * \param required_extruder_prime The pre-computed list of extruders uses during this layer
     * \param method The current prime tower strategy
     * \param initial_list_idx A list potentially containing extruders that we already know can be used for
     *                         sparse infill
     * \return The indexes of extruders to be used for sparse infill
     */
    std::vector<size_t> findExtrudersSparseInfill(
        LayerPlan& gcode_layer,
        const std::vector<ExtruderUse>& required_extruder_prime,
        cura::PrimeTowerMethod method,
        const std::vector<size_t>& initial_list_idx = {}) const;

    /*!
     * For an extruder switch that happens not on the first layer, the extruder needs to be primed on the prime tower.
     * This function picks a start location for this extruder on the prime tower's perimeter and travels there to avoid
     * starting at the location everytime which can result in z-seam blobs.
     */
    void gotoStartLocation(LayerPlan& gcode_layer, const int extruder) const;
};


} // namespace cura

#endif // PRIME_TOWER_H
