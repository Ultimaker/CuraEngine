// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PRIME_TOWER_H
#define PRIME_TOWER_H

#include <map>
#include <vector>

#include "ExtruderUse.h"
#include "geometry/ClosedLinesSet.h"
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
 * Abstract class for everything to do with the prime tower:
 * - Generating the occupation areas.
 * - Checking up untill which height the prime tower has to be printed.
 * - Inserting priming commands in extruders uses
 * - Generating priming paths and adding them to the layer plan.
 *
 * We may adopt different strategies to generate the prime tower, thus this class is abstract and different
 * implementations may co-exist. The common behavior implemented in the main class is:
 * - Generate occupation areas as a cylinder with a flared base
 * - Generate the base extra extrusion discs around the base cylinder
 * - Generate the first layer extra inset inside the base cylinder
 * Then it is the job of the specific implementation to handle the generation of extrusion paths for the base cylinder
 */
class PrimeTower
{
protected:
    struct OccupiedOutline
    {
        Polygon outline;
        coord_t outer_radius;
    };

    struct ExtruderToolPaths
    {
        size_t extruder_nr;
        ClosedLinesSet toolpaths;
        coord_t outer_radius;
        coord_t inner_radius;
    };

private:
    bool wipe_from_middle_; //!< Whether to wipe on the inside of the hollow prime tower
    Point2LL middle_; //!< The middle of the prime tower

    Point2LL post_wipe_point_; //!< Location to post-wipe the unused nozzle off on

    static constexpr size_t number_of_prime_tower_start_locations_ = 21; //!< The required size of \ref PrimeTower::wipe_locations
    inline static const AngleRadians start_locations_step_ = (std::numbers::pi * 2.0) / number_of_prime_tower_start_locations_;

    /*
     *  The map index is the layer number
     *  For each layer, the list contains the extruders moves to be processed. This list is sorted from outer annuli to inner
     *  annuli, which is not the printing chronological order, but the physical arrangement.
     */
    std::map<LayerIndex, std::vector<ExtruderToolPaths>> toolpaths_;

    OccupiedOutline outer_poly_; //!< The outline of the prime tower, not including the base

    //!< This is the exact outline of the extrusions lines of each layer, for layers having extra width for the base
    LayerVector<Polygon> base_extrusion_outline_;
    //!< This is the approximate outline of the area filled at each layer, for layers having extra width for the base
    LayerVector<OccupiedOutline> base_occupied_outline_;

    static constexpr size_t circle_definition_{ 32 }; // The number of vertices in each circle.
    static constexpr size_t arc_definition_{ 4 }; // The number of segments in each arc of a wheel

public:
    /*! \brief Creates a prime tower instance that will determine where and how the prime tower gets printed. */
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
    const Polygon& getOccupiedOutline(const LayerIndex& layer_nr) const;

    /*!
     * Get the occupied outline of the prime tower at the first layer
     *
     * \note @sa getOccupiedOutline()
     */
    const Polygon& getOccupiedGroundOutline() const;

    /*!
     * Get the extrusion outline of the prime tower at the given layer
     *
     * \param[in] layer_nr The index of the layer
     * \return The extrusion outline for the prime tower at the given layer
     * \note The returned outline is the exact outline of the extrusion path, which is useful if you need to generate a toolpath
     *       touching the prime tower. Otherwise use getExtrusionOutline(). This method will return the valid result only after
     *       processExtrudersUse() has been called, which is "late" is the global slicing operation.
     */
    const Polygon& getExtrusionOutline(const LayerIndex& layer_nr) const;

    /*!
     * \brief Get the required priming for the given extruder at the given layer
     * \param extruder_is_used_on_this_layer A list indicating which extruders are used at this layer
     * \param extruder_nr The extruder for which we want the priming information
     * \param last_extruder The extruder that was in use just before using the new one
     * \param storage The storage containing all the slice data
     * \param layer_nr The layer at which we want to use the extruder
     * \return An enumeration indication how the extruder will be used by the prime tower at this layer
     */
    virtual ExtruderPrime getExtruderPrime(
        const std::vector<bool>& extruder_is_used_on_this_layer,
        size_t extruder_nr,
        size_t last_extruder,
        const SliceDataStorage& storage,
        const LayerIndex& layer_nr) const = 0;

    /*!
     * \brief This method has to be called once the extruders use for each layer have been calculated. From this point,
     *        we can start generating the prime tower, and also polish the given extruders uses.
     * \param extruders_use The calculated extruders uses at each layer. This may be slightly changed to make sure that
     *                      the prime tower can be properly printed.
     * \param start_extruder The very first used extruder
     */
    void processExtrudersUse(LayerVector<std::vector<ExtruderUse>>& extruders_use, const size_t start_extruder);

    /*!
     * \brief Create the proper prime tower object according to the current settings
     * \param storage The storage containing all the slice data
     * \return The proper prime tower object, which may be null if prime tower is actually disabled or not required
     */
    static PrimeTower* createPrimeTower(SliceDataStorage& storage);

protected:
    /*!
     * \brief Once all the extruders uses have been calculated for each layer, this method makes a global pass to make
     *        sure that the prime tower can be properly printed. This is required because we sometimes need to know what
     *        a layer above is made of to fix a layer below.
     * \param extruders_use The calculated extruders uses at each layer
     * \param start_extruder The very first used extruder
     */
    virtual void polishExtrudersUses(LayerVector<std::vector<ExtruderUse>>& /*extruders_use*/, const size_t /*start_extruder*/)
    {
        // Default behavior is to keep the extruders uses as they were calculated
    }

    /*!
     * \brief Generated the extruders toolpaths for each layer of the prime tower
     * \param extruders_use The calculated extruders uses at each layer
     * \return A map of extruders toolpaths per layer. The inner list is sorted from outer annuli to inner
     *         annuli, which is not the printing chronological order, but the physical arrangement. @sa toolpaths_
     */
    virtual std::map<LayerIndex, std::vector<ExtruderToolPaths>> generateToolPaths(const LayerVector<std::vector<ExtruderUse>>& extruders_use) = 0;

    /*!
     * \brief Generate the actual priming toolpaths for the given extruder, starting at the given outer circle radius
     * \param extruder_nr The extruder for which we want the priming toolpath
     * \param outer_radius The radius of the starting outer circle
     * \return A tuple containing the newly generated toolpaths, and the inner radius of the newly generated annulus
     */
    std::tuple<ClosedLinesSet, coord_t> generatePrimeToolpaths(const size_t extruder_nr, const coord_t outer_radius);

    /*!
     * \brief Generate support toolpaths using the wheel pattern applied on an annulus
     * \param extruder_nr The extruder for which we want the support toolpath
     * \param outer_radius The annulus outer radius
     * \param inner_radius The annulis inner radius
     * \return
     */
    ClosedLinesSet generateSupportToolpaths(const size_t extruder_nr, const coord_t outer_radius, const coord_t inner_radius);

    /*!
     * \brief Calculates whether an extruder requires priming at a specific layer
     * \param extruder_is_used_on_this_layer The list of used extruders at this layer
     * \param extruder_nr The extruder we now want to use
     * \param last_extruder The extruder that was in use before switching to the new one
     * \return True if the extruder needs to be primed, false otherwise
     */
    static bool extruderRequiresPrime(const std::vector<bool>& extruder_is_used_on_this_layer, size_t extruder_nr, size_t last_extruder);

private:
    /*! \brief Generates the extra inset used for better adhesion at the first layer */
    void generateFirtLayerInset();

    /*! \brief Generates the extra annuli around the first layers of the prime tower which help make it stronger */
    void generateBase();

    /*!
     * For an extruder switch that happens not on the first layer, the extruder needs to be primed on the prime tower.
     * This function picks a start location for this extruder on the prime tower's perimeter and travels there to avoid
     * starting at the location everytime which can result in z-seam blobs.
     */
    void gotoStartLocation(LayerPlan& gcode_layer, const size_t extruder) const;

    /*!
     * \brief Subtract the prime tower from the support areas in storage.
     * \param storage The storage where to find the support from which to subtract a prime tower.
     */
    void subtractFromSupport(SliceDataStorage& storage);
};

} // namespace cura

#endif // PRIME_TOWER_H
