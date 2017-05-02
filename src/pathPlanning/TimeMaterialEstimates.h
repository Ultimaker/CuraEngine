/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef PATH_PLANNING_TIME_MATERIAL_ESTIMATES_H
#define PATH_PLANNING_TIME_MATERIAL_ESTIMATES_H

#include "../gcodeExport.h"

namespace cura 
{

class ExtruderPlan; // forward declaration so that TimeMaterialEstimates can be a friend

/*!
 * Time and material estimates for a portion of paths, e.g. layer, extruder plan, path.
 */
class TimeMaterialEstimates
{
    friend class ExtruderPlan; // cause there the naive estimates are calculated
private:
    double extrude_time; //!< Time in seconds occupied by extrusion
    double unretracted_travel_time; //!< Time in seconds occupied by non-retracted travel (non-extrusion)
    double retracted_travel_time; //!< Time in seconds occupied by retracted travel (non-extrusion)
    double material; //!< Material used (in mm^3)
public:
    /*!
     * Basic contructor
     * 
     * \param extrude_time Time in seconds occupied by extrusion
     * \param unretracted_travel_time Time in seconds occupied by non-retracted travel (non-extrusion)
     * \param retracted_travel_time Time in seconds occupied by retracted travel (non-extrusion)
     * \param material Material used (in mm^3)
     */
    TimeMaterialEstimates(double extrude_time, double unretracted_travel_time, double retracted_travel_time, double material);

    /*!
     * Basic constructor initializing all estimates to zero.
     */
    TimeMaterialEstimates();

    /*!
     * Set all estimates to zero.
     */
    void reset();

    /*!
     * Pointwise addition of estimate stats
     * 
     * \param other The estimates to add to these estimates.
     * \return The resulting estimates
     */
    TimeMaterialEstimates operator+(const TimeMaterialEstimates& other);

    /*!
     * In place pointwise addition of estimate stats
     * 
     * \param other The estimates to add to these estimates.
     * \return These estimates
     */
    TimeMaterialEstimates& operator+=(const TimeMaterialEstimates& other);

    /*!
     * \brief Subtracts the specified estimates from these estimates and returns
     * the result.
     * 
     * Each of the estimates in this class are individually subtracted.
     * 
     * \param other The estimates to subtract from these estimates.
     * \return These estimates with the specified estimates subtracted.
     */
    TimeMaterialEstimates operator-(const TimeMaterialEstimates& other);

    /*!
     * \brief Subtracts the specified elements from these estimates.
     * 
     * This causes the estimates in this instance to change. Each of the
     * estimates in this class are individually subtracted.
     * 
     * \param other The estimates to subtract from these estimates.
     * \return A reference to this instance.
     */
    TimeMaterialEstimates& operator-=(const TimeMaterialEstimates& other);

    /*!
     * Get total time estimate. The different time estimate member values added together.
     * 
     * \return the total of all different time estimate values 
     */
    double getTotalTime() const;

    /*!
     * Get the total time during which the head is not retracted.
     * 
     * This includes extrusion time and non-retracted travel time
     * 
     * \return the total time during which the head is not retracted.
     */
    double getTotalUnretractedTime() const;

    /*!
     * Get the total travel time.
     * 
     * This includes the retracted travel time as well as the unretracted travel time.
     * 
     * \return the total travel time.
     */
    double getTravelTime() const;

    /*!
     * Get the extrusion time.
     * 
     * \return extrusion time.
     */
    double getExtrudeTime() const;

    /*!
     * Get the amount of material used in mm^3.
     * 
     * \return amount of material
     */
    double getMaterial() const;
};

}//namespace cura

#endif//PATH_PLANNING_TIME_MATERIAL_ESTIMATES_H
