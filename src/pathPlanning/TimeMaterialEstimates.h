/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef PATH_PLANNING_TIME_MATERIAL_ESTIMATES_H
#define PATH_PLANNING_TIME_MATERIAL_ESTIMATES_H

#include "../gcodeExport.h"

namespace cura 
{

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
    TimeMaterialEstimates(double extrude_time, double unretracted_travel_time, double retracted_travel_time, double material)
    : extrude_time(extrude_time)
    , unretracted_travel_time(unretracted_travel_time)
    , retracted_travel_time(retracted_travel_time)
    , material(material)
    {
    }

    /*!
     * Basic constructor initializing all estimates to zero.
     */
    TimeMaterialEstimates()
    : extrude_time(0.0)
    , unretracted_travel_time(0.0)
    , retracted_travel_time(0.0)
    , material(0.0)
    {
    }

    /*!
     * Set all estimates to zero.
     */
    void reset() 
    {
        extrude_time = 0.0;
        unretracted_travel_time = 0.0;
        retracted_travel_time = 0.0;
        material = 0.0;
    }

    /*!
     * Pointwise addition of estimate stats
     * 
     * \param other The estimates to add to these estimates.
     * \return The resulting estimates
     */
    TimeMaterialEstimates operator+(const TimeMaterialEstimates& other)
    {
        return TimeMaterialEstimates(extrude_time+other.extrude_time, unretracted_travel_time+other.unretracted_travel_time, retracted_travel_time+other.retracted_travel_time, material+other.material);
    }

    /*!
     * In place pointwise addition of estimate stats
     * 
     * \param other The estimates to add to these estimates.
     * \return These estimates
     */
    TimeMaterialEstimates& operator+=(const TimeMaterialEstimates& other)
    {
        extrude_time += other.extrude_time;
        unretracted_travel_time += other.unretracted_travel_time;
        retracted_travel_time += other.retracted_travel_time;
        material += other.material;
        return *this;
    }

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
    double getTotalTime() const
    {
        return extrude_time + unretracted_travel_time + retracted_travel_time;
    }

    /*!
     * Get the total time during which the head is not retracted.
     * 
     * This includes extrusion time and non-retracted travel time
     * 
     * \return the total time during which the head is not retracted.
     */
    double getTotalUnretractedTime() const
    {
        return extrude_time + unretracted_travel_time;
    }

    /*!
     * Get the total travel time.
     * 
     * This includes the retracted travel time as well as the unretracted travel time.
     * 
     * \return the total travel time.
     */
    double getTravelTime() const
    {
        return retracted_travel_time + unretracted_travel_time;
    }

    /*!
     * Get the extrusion time.
     * 
     * \return extrusion time.
     */
    double getExtrudeTime() const
    {
        return extrude_time;
    }

    /*!
     * Get the amount of material used in mm^3.
     * 
     * \return amount of material
     */
    double getMaterial() const
    {
        return material;
    }
};

}//namespace cura

#endif//PATH_PLANNING_TIME_MATERIAL_ESTIMATES_H
