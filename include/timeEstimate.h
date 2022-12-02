//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TIME_ESTIMATE_H
#define TIME_ESTIMATE_H

#include <stdint.h>
#include <vector>
#include <unordered_map>

#include "PrintFeature.h"
#include "settings/types/Duration.h" //Print time estimates.
#include "settings/types/Velocity.h" //Speeds and accelerations at which we print.

namespace cura
{

class Ratio;
class Settings;

/*!
 *  The TimeEstimateCalculator class generates a estimate of printing time calculated with acceleration in mind.
 *  Some of this code has been adapted from the Marlin sources.
 */

class TimeEstimateCalculator
{
public:
    constexpr static unsigned int NUM_AXIS = 4;
    constexpr static unsigned int X_AXIS = 0;
    constexpr static unsigned int Y_AXIS = 1;
    constexpr static unsigned int Z_AXIS = 2;
    constexpr static unsigned int E_AXIS = 3;


    class Position
    {
    public:
        Position() {for(unsigned int n=0;n<NUM_AXIS;n++) axis[n] = 0;}
        Position(double x, double y, double z, double e) {axis[0] = x;axis[1] = y;axis[2] = z;axis[3] = e;}
        double axis[NUM_AXIS];
        
        double& operator[](const int n) { return axis[n]; }
    };

    class Block
    {
    public:
        bool recalculate_flag;
        
        double accelerate_until;
        double decelerate_after;
        Velocity initial_feedrate;
        Velocity final_feedrate;

        Velocity entry_speed;
        Velocity max_entry_speed;
        bool nominal_length_flag;
        
        Velocity nominal_feedrate;
        double maxTravel;
        double distance;
        Acceleration acceleration;
        Position delta;
        Position absDelta;

        PrintFeatureType feature;
    };

private:
    Velocity max_feedrate[NUM_AXIS] = {600, 600, 40, 25}; // mm/s
    Velocity minimumfeedrate = 0.01;
    Acceleration acceleration = 3000;
    Acceleration max_acceleration[NUM_AXIS] = {9000, 9000, 100, 10000};
    Velocity max_xy_jerk = 20.0;
    Velocity max_z_jerk = 0.4;
    Velocity max_e_jerk = 5.0;
    Duration extra_time = 0.0;
    
    Position previous_feedrate;
    Velocity previous_nominal_feedrate;

    Position currentPosition;

    std::vector<Block> blocks;
public:
    /*!
     * \brief Set the movement configuration of the firmware.
     * \param settings_base Where to get the settings from.
     */
    void setFirmwareDefaults(const Settings& settings);
    void setPosition(Position newPos);
    void plan(Position newPos, Velocity feedRate, PrintFeatureType feature);
    void addTime(const Duration& time);
    void setAcceleration(const Acceleration& acc); //!< Set the default acceleration to \p acc
    void setMaxXyJerk(const Velocity& jerk); //!< Set the max xy jerk to \p jerk

    void reset();
    
    std::vector<Duration> calculate();
private:
    void reversePass();
    void forwardPass();

    // Recalculates the trapezoid speed profiles for all blocks in the plan according to the
    // entry_factor for each junction. Must be called by planner_recalculate() after
    // updating the blocks.
    void recalculateTrapezoids();

    // Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
    void calculateTrapezoidForBlock(Block *block, const Ratio entry_factor, const Ratio exit_factor);

    // The kernel called by accelerationPlanner::calculate() when scanning the plan from last to first entry.
    void plannerReversePassKernel(Block *previous, Block *current, Block *next);

    // The kernel called by accelerationPlanner::calculate() when scanning the plan from first to last entry.
    void plannerForwardPassKernel(Block *previous, Block *current, Block *next);
};

}//namespace cura
#endif//TIME_ESTIMATE_H
