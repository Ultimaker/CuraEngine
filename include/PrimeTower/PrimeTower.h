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
#include "utils/LayerVector.h"
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
protected:
    struct ExtruderMoves
    {
        size_t extruder_nr;
        Shape moves;
    };

private:
    using MovesByLayer = std::map<size_t, LayerVector<Shape>>;

    bool wipe_from_middle_; //!< Whether to wipe on the inside of the hollow prime tower
    Point2LL middle_; //!< The middle of the prime tower

    Point2LL post_wipe_point_; //!< Location to post-wipe the unused nozzle off on

    std::vector<ClosestPointPolygon> prime_tower_start_locations_; //!< The differernt locations where to pre-wipe the active nozzle
    const unsigned int number_of_prime_tower_start_locations_ = 21; //!< The required size of \ref PrimeTower::wipe_locations

    /*
     *  The map index is the layer number
     *  For each layer, the list contains the extruders moves to be processed. This list is sorted from outer rings to inner
     *  rings, which is not the printing chronological order, but the physical arrangement.
     */
    std::map<LayerIndex, std::vector<ExtruderMoves>> moves_;

    Shape outer_poly_; //!< The outline of the prime tower, not including the base

    //!< This is the exact outline of the extrusions lines of each layer, for layers having extra width for the base
    LayerVector<Shape> base_extrusion_outline_;
    //!< This is the approximate outline of the area filled at each layer, for layers having extra width for the base
    LayerVector<Shape> base_occupied_outline_;

public:
    /*!
     * \brief Creates a prime tower instance that will determine where and how
     * the prime tower gets printed.
     *
     * \param storage A storage where it retrieves the prime tower settings.
     */
    PrimeTower();

    virtual ~PrimeTower() = default;

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
     * Get the occupied outline of the prime tower at the given layer
     *
     * \param[in] layer_nr The index of the layer
     * \return The outer polygon for the prime tower at the given layer
     * \note The returned outline is a close approximation of the actual toolpaths. The actual extrusion area may be slightly smaller.
     *       Use this method only if you need to get the exclusion area of the prime tower. Otherwise use getExtrusionOutline().
     *       This method exists because this approximate area can be calculated as soon as the prime tower is initialized.
     */
    const Shape& getOccupiedOutline(const LayerIndex& layer_nr) const;

    /*!
     * Get the occupied outline of the prime tower at the first layer
     *
     * \note @sa getOccupiedOutline()
     */
    const Shape& getOccupiedGroundOutline() const;

    /*!
     * Get the extrusion outline of the prime tower at the given layer
     *
     * \param[in] layer_nr The index of the layer
     * \return The extrusion outline for the prime tower at the given layer
     * \note The returned outline is the exact outline of the extrusion path, which is useful if you need to generate a toolpath
     *       touching the prime tower. Otherwise use getExtrusionOutline(). This method will return the valid result only after
     *       processExtrudersUse() has been called, which is "late" is the global slicing operation.
     */
    const Shape& getExtrusionOutline(const LayerIndex& layer_nr) const;

    virtual ExtruderPrime getExtruderPrime(
        const std::vector<bool>& extruder_is_used_on_this_layer,
        size_t extruder_nr,
        size_t last_extruder,
        const SliceDataStorage& storage,
        const LayerIndex& layer_nr) const = 0;

    void processExtrudersUse(LayerVector<std::vector<ExtruderUse>>& extruders_use, const SliceDataStorage& storage, const size_t start_extruder);

    static PrimeTower* createPrimeTower(SliceDataStorage& storage);

protected:
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
        const size_t actual_extruder_nr) const;

    virtual void polishExtrudersUses(LayerVector<std::vector<ExtruderUse>>& /*extruders_use*/, const SliceDataStorage& /*storage*/, const size_t /*start_extruder*/)
    {
    }

    virtual std::map<LayerIndex, std::vector<ExtruderMoves>> generateExtrusionsMoves(const LayerVector<std::vector<ExtruderUse>>& extruders_use, const SliceDataStorage& storage)
        = 0;

    std::tuple<Shape, coord_t> generatePrimeMoves(const size_t extruder_nr, const coord_t outer_radius);

    Shape generateSupportMoves(const size_t extruder_nr, const coord_t outer_radius, const coord_t inner_radius);

    static bool extruderRequiresPrime(const std::vector<bool>& extruder_is_used_on_this_layer, size_t extruder_nr, size_t last_extruder);

private:
    void generateFirtLayerInset();

    /*!
     * For an extruder switch that happens not on the first layer, the extruder needs to be primed on the prime tower.
     * This function picks a start location for this extruder on the prime tower's perimeter and travels there to avoid
     * starting at the location everytime which can result in z-seam blobs.
     */
    void gotoStartLocation(LayerPlan& gcode_layer, const size_t extruder) const;

    /*!
     * \brief Subtract the prime tower from the support areas in storage.
     *
     * \param storage The storage where to find the support from which to
     * subtract a prime tower.
     */
    void subtractFromSupport(SliceDataStorage& storage);

    void generateBase();
};

} // namespace cura

#endif // PRIME_TOWER_H
