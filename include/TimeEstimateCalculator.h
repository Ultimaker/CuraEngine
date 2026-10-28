// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <cstdint>
#include <vector>

#include "PrintFeatureType.h"
#include "types/Acceleration.h"
#include "types/Duration.h" //Print time estimates.
#include "types/Ratio.h"
#include "types/Velocity.h" //Speeds and accelerations at which we print.

namespace curaengine
{

class Settings;

/*!
 *  The TimeEstimateCalculator class generates a estimate of printing time calculated with acceleration in mind.
 *  Some of this code has been adapted from the Marlin sources.
 */

class TimeEstimateCalculator
{
public:
    constexpr static size_t NUM_AXIS = 4;
    constexpr static size_t X_AXIS = 0;
    constexpr static size_t Y_AXIS = 1;
    constexpr static size_t Z_AXIS = 2;
    constexpr static size_t E_AXIS = 3;


    class Position
    {
    public:
        Position()
        {
            for (unsigned int n = 0; n < NUM_AXIS; n++)
                axis[n] = 0;
        }
        Position(double x, double y, double z, double e)
        {
            axis[0] = x;
            axis[1] = y;
            axis[2] = z;
            axis[3] = e;
        }
        double axis[NUM_AXIS];

        double& operator[](const size_t n)
        {
            return axis[n];
        }
    };

    class Block
    {
    public:
        bool recalculate_flag;

        double accelerate_until;
        double decelerate_after;
        types::Velocity initial_feedrate;
        types::Velocity final_feedrate;

        types::Velocity entry_speed;
        types::Velocity max_entry_speed;
        bool nominal_length_flag;

        types::Velocity nominal_feedrate;
        double maxTravel;
        double distance;
        types::Acceleration acceleration;
        Position delta;
        Position absDelta;

        PrintFeatureType feature;
    };

private:
    types::Velocity max_feedrate[NUM_AXIS] = { 600.0, 600.0, 40.0, 25.0 }; // mm/s
    types::Velocity minimumfeedrate = 0.01;
    types::Acceleration acceleration = 3000.0;
    types::Acceleration max_acceleration[NUM_AXIS] = { 9000.0, 9000.0, 100.0, 10000.0 };
    types::Velocity max_xy_jerk = 20.0;
    types::Velocity max_z_jerk = 0.4;
    types::Velocity max_e_jerk = 5.0;
    types::Duration extra_time = 0.0;

    Position previous_feedrate;
    types::Velocity previous_nominal_feedrate;

    Position currentPosition;

    std::vector<Block> blocks;

public:
    /*!
     * \brief Set the movement configuration of the firmware.
     * \param settings_base Where to get the settings from.
     */
    void setFirmwareDefaults(const Settings& settings);
    void setPosition(Position newPos);
    void plan(Position newPos, types::Velocity feedRate, PrintFeatureType feature);
    void addTime(const types::Duration& time);
    void setAcceleration(const types::Acceleration& acc); //!< Set the default acceleration to \p acc
    void setMaxXyJerk(const types::Velocity& jerk); //!< Set the max xy jerk to \p jerk

    void reset();

    std::vector<types::Duration> calculate();

private:
    void reversePass();
    void forwardPass();

    // Recalculates the trapezoid speed profiles for all blocks in the plan according to the
    // entry_factor for each junction. Must be called by planner_recalculate() after
    // updating the blocks.
    void recalculateTrapezoids();

    // Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
    void calculateTrapezoidForBlock(Block* block, const types::Ratio entry_factor, const types::Ratio exit_factor);

    // The kernel called by accelerationPlanner::calculate() when scanning the plan from last to first entry.
    void plannerReversePassKernel(Block* previous, Block* current, Block* next);

    // The kernel called by accelerationPlanner::calculate() when scanning the plan from first to last entry.
    void plannerForwardPassKernel(Block* previous, Block* current, Block* next);
};

} // namespace curaengine
