//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>

#include "timeEstimate.h"
#include "utils/math.h"
#include "settings/Settings.h"
#include "settings/types/Ratio.h"

namespace cura
{

#define MINIMUM_PLANNER_SPEED 0.05 // mm/sec

void TimeEstimateCalculator::setFirmwareDefaults(const Settings& settings)
{
    max_feedrate[X_AXIS] = settings.get<Velocity>("machine_max_feedrate_x");
    max_feedrate[Y_AXIS] = settings.get<Velocity>("machine_max_feedrate_y");
    max_feedrate[Z_AXIS] = settings.get<Velocity>("machine_max_feedrate_z");
    max_feedrate[E_AXIS] = settings.get<Velocity>("machine_max_feedrate_e");
    max_acceleration[X_AXIS] = settings.get<Acceleration>("machine_max_acceleration_x");
    max_acceleration[Y_AXIS] = settings.get<Acceleration>("machine_max_acceleration_y");
    max_acceleration[Z_AXIS] = settings.get<Acceleration>("machine_max_acceleration_z");
    max_acceleration[E_AXIS] = settings.get<Acceleration>("machine_max_acceleration_e");
    max_xy_jerk = settings.get<Velocity>("machine_max_jerk_xy");
    max_z_jerk = settings.get<Velocity>("machine_max_jerk_z");
    max_e_jerk = settings.get<Velocity>("machine_max_jerk_e");
    minimumfeedrate = settings.get<Velocity>("machine_minimum_feedrate");
    acceleration = settings.get<Acceleration>("machine_acceleration");
}


void TimeEstimateCalculator::setPosition(Position newPos)
{
    currentPosition = newPos;
}

void TimeEstimateCalculator::addTime(const Duration& time)
{
    extra_time += time;
}

void TimeEstimateCalculator::setAcceleration(const Velocity& acc)
{
    acceleration = acc;
}

void TimeEstimateCalculator::setMaxXyJerk(const Velocity& jerk)
{
    max_xy_jerk = jerk;
}

void TimeEstimateCalculator::reset()
{
    extra_time = 0.0;
    blocks.clear();
}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
static inline Velocity maxAllowableSpeed(const Acceleration& acceleration, const Velocity& target_velocity, double distance)
{
    return sqrt(target_velocity * target_velocity - 2 * acceleration * distance);
}

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the given acceleration:
static inline float estimateAccelerationDistance(const Velocity& initial_rate, const Velocity& target_rate, const Acceleration& acceleration)
{
    if (acceleration == 0)
    {
        return 0.0;
    }
    return (square(target_rate) - square(initial_rate)) / (2.0 * acceleration);
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)
static inline double intersectionDistance(const Velocity& initial_rate, const Velocity& final_rate, const Acceleration& acceleration, double distance)
{
    if (acceleration == 0.0)
    {
        return 0.0;
    }

    /*
     * Calculate the intersection point of two time-velocity formulas: One for
     * accelerating and one for decelerating.
     * Imagine a graph that plots velocity [v] against position [d] (different
     * from the other code in this class, which compares velocity with time).
     * In this graph, the accelerating part will have the formula:
     * v = sqrt(2ad + v_i²)
     * Where:
     * * v := velocity
     * * a := acceleration
     * * d := distance (the unknown variable)
     * * v_i := initial velocity
     * Similarly, the decelerating part will have the formula:
     * v = sqrt(2a(D - d) + v_f²)
     * Where:
     * * D := total line length
     * * v_f := final velocity at the end of the line
     * To find the position where we need to start decelerating, simply find the
     * position where the velocity in these formulas is the same. In other words
     * solve this formula for d:
     * sqrt(2ad + v_i²) = sqrt(2a(D - d) + v_f²)
     * 2ad + v_i² = 2a(D - d) + v_f² [square both sides]
     * 2ad - 2a(D - d) = v_f² - v_i² [+v_i², -2a(D - d) on both sides]
     * d - (D - d) = (v_f² - v_i²) / 2a [divide by 2a on both sides, expand brackets]
     * 2d - D = (v_f² - v_i²) / 2a [expand brackets on the left]
     * 2d = (2aD + v_f² - v_i²) / 2a [+D on both sides, but on the right multiply it by 2a to put it in the brackets]
     * d = (2aD + v_f² - v_i²) / 4a [divide by 2 on both sides]
     */
    return (2.0 * acceleration * distance - square(initial_rate) + square(final_rate)) / (4.0 * acceleration);
}

// This function gives the time it needs to accelerate from an initial speed to reach a final distance.
static inline double accelerationTimeFromDistance(const Velocity& initial_feedrate, const Velocity& distance, const Acceleration& acceleration)
{
    double discriminant = square(initial_feedrate) - 2 * acceleration * -distance;
    //If discriminant is negative, we're moving in the wrong direction.
    //Making the discriminant 0 then gives the extremum of the parabola instead of the intersection.
    discriminant = std::max(0.0, discriminant);
    return (-initial_feedrate + sqrt(discriminant)) / acceleration;
}

void TimeEstimateCalculator::calculateTrapezoidForBlock(Block *block, const Ratio entry_factor, const Ratio exit_factor)
{
    const Velocity initial_feedrate = block->nominal_feedrate * entry_factor;
    const Velocity final_feedrate = block->nominal_feedrate * exit_factor;

    double accelerate_distance = estimateAccelerationDistance(initial_feedrate, block->nominal_feedrate, block->acceleration);
    const double decelerate_distance = estimateAccelerationDistance(block->nominal_feedrate, final_feedrate, -block->acceleration);

    // Calculate the size of Plateau of Nominal Rate.
    double plateau_distance = block->distance-accelerate_distance - decelerate_distance;

    // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
    // have to use intersection_distance() to calculate when to abort acceleration and start braking
    // in order to reach the final_rate exactly at the end of this block.
    if (plateau_distance < 0)
    {
        accelerate_distance = intersectionDistance(initial_feedrate, final_feedrate, block->acceleration, block->distance);
        accelerate_distance = std::max(accelerate_distance, 0.0); // Check limits due to numerical round-off
        accelerate_distance = std::min(accelerate_distance, block->distance);//(We can cast here to unsigned, because the above line ensures that we are above zero)
        plateau_distance = 0;
    }

    block->accelerate_until = accelerate_distance;
    block->decelerate_after = accelerate_distance + plateau_distance;
    block->initial_feedrate = initial_feedrate;
    block->final_feedrate = final_feedrate;
}

void TimeEstimateCalculator::plan(Position newPos, Velocity feedrate, PrintFeatureType feature)
{
    Block block;
    memset(&block, 0, sizeof(block));

    block.feature = feature;

    //block.maxTravel = 0; //Done by memset.
    for(size_t n = 0; n < NUM_AXIS; n++)
    {
        block.delta[n] = newPos[n] - currentPosition[n];
        block.absDelta[n] = fabs(block.delta[n]);
        block.maxTravel = std::max(block.maxTravel, block.absDelta[n]);
    }
    if (block.maxTravel <= 0)
    {
        return;
    }
    if (feedrate < minimumfeedrate)
    {
        feedrate = minimumfeedrate;
    }
    block.distance = sqrtf(square(block.absDelta[0]) + square(block.absDelta[1]) + square(block.absDelta[2]));
    if (block.distance == 0.0)
    {
        block.distance = block.absDelta[3];
    }
    block.nominal_feedrate = feedrate;
    
    Position current_feedrate;
    Position current_abs_feedrate;
    Ratio feedrate_factor = 1.0;
    for(size_t n = 0; n < NUM_AXIS; n++)
    {
        current_feedrate[n] = (block.delta[n] * feedrate) / block.distance;
        current_abs_feedrate[n] = fabs(current_feedrate[n]);
        if (current_abs_feedrate[n] > max_feedrate[n])
        {
            feedrate_factor = std::min(feedrate_factor, Ratio(max_feedrate[n] / current_abs_feedrate[n]));
        }
    }
    //TODO: XY_FREQUENCY_LIMIT
    
    if (feedrate_factor < 1.0)
    {
        for(size_t n = 0; n < NUM_AXIS; n++)
        {
            current_feedrate[n] *= feedrate_factor;
            current_abs_feedrate[n] *= feedrate_factor;
        }
        block.nominal_feedrate *= feedrate_factor;
    }
    
    block.acceleration = acceleration;
    for(size_t n = 0; n < NUM_AXIS; n++)
    {
        if (block.acceleration * (block.absDelta[n] / block.distance) > max_acceleration[n])
        {
            block.acceleration = max_acceleration[n];
        }
    }
    
    Velocity vmax_junction = max_xy_jerk / 2;
    Ratio vmax_junction_factor = 1.0;
    if (current_abs_feedrate[Z_AXIS] > max_z_jerk / 2)
    {
        vmax_junction = std::min(vmax_junction, max_z_jerk / 2);
    }
    if (current_abs_feedrate[E_AXIS] > max_e_jerk / 2)
    {
        vmax_junction = std::min(vmax_junction, max_e_jerk / 2);
    }
    vmax_junction = std::min(vmax_junction, block.nominal_feedrate);
    const Velocity safe_speed = vmax_junction;
    
    if ((blocks.size() > 0) && (previous_nominal_feedrate > 0.0001))
    {
        const Velocity xy_jerk = sqrt(square(current_feedrate[X_AXIS] - previous_feedrate[X_AXIS]) + square(current_feedrate[Y_AXIS] - previous_feedrate[Y_AXIS]));
        vmax_junction = block.nominal_feedrate;
        if (xy_jerk > max_xy_jerk)
        {
            vmax_junction_factor = Ratio(max_xy_jerk / xy_jerk);
        } 
        if (fabs(current_feedrate[Z_AXIS] - previous_feedrate[Z_AXIS]) > max_z_jerk)
        {
            vmax_junction_factor = std::min(vmax_junction_factor, Ratio(max_z_jerk / fabs(current_feedrate[Z_AXIS] - previous_feedrate[Z_AXIS])));
        } 
        if (fabs(current_feedrate[E_AXIS] - previous_feedrate[E_AXIS]) > max_e_jerk)
        {
            vmax_junction_factor = std::min(vmax_junction_factor, Ratio(max_e_jerk / fabs(current_feedrate[E_AXIS] - previous_feedrate[E_AXIS])));
        }
        vmax_junction = std::min(previous_nominal_feedrate, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed
    }

    block.max_entry_speed = vmax_junction;

    const Velocity v_allowable = maxAllowableSpeed(-block.acceleration, MINIMUM_PLANNER_SPEED, block.distance);
    block.entry_speed = std::min(vmax_junction, v_allowable);
    block.nominal_length_flag = block.nominal_feedrate <= v_allowable;
    block.recalculate_flag = true; // Always calculate trapezoid for new block

    previous_feedrate = current_feedrate;
    previous_nominal_feedrate = block.nominal_feedrate;

    currentPosition = newPos;

    calculateTrapezoidForBlock(&block, Ratio(block.entry_speed / block.nominal_feedrate), Ratio(safe_speed / block.nominal_feedrate));

    blocks.push_back(block);
}

std::vector<Duration> TimeEstimateCalculator::calculate()
{
    reversePass();
    forwardPass();
    recalculateTrapezoids();
    
    std::vector<Duration> totals(static_cast<unsigned char>(PrintFeatureType::NumPrintFeatureTypes), 0.0);
    totals[static_cast<unsigned char>(PrintFeatureType::NoneType)] = extra_time; // Extra time (pause for minimum layer time, etc) is marked as NoneType
    for(unsigned int n = 0; n < blocks.size(); n++)
    {
        const Block& block = blocks[n];
        const double plateau_distance = block.decelerate_after - block.accelerate_until;

        totals[static_cast<unsigned char>(block.feature)] += accelerationTimeFromDistance(block.initial_feedrate, block.accelerate_until, block.acceleration);
        totals[static_cast<unsigned char>(block.feature)] += plateau_distance / block.nominal_feedrate;
        totals[static_cast<unsigned char>(block.feature)] += accelerationTimeFromDistance(block.final_feedrate, (block.distance - block.decelerate_after), block.acceleration);
    }
    return totals;
}

void TimeEstimateCalculator::plannerReversePassKernel(Block *previous, Block *current, Block *next)
{
    (void)previous;
    if (!current || !next)
    {
        return;
    }

    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (current->entry_speed != current->max_entry_speed)
    {
        // If nominal length true, max junction speed is guaranteed to be reached. Only compute
        // for max allowable speed if block is decelerating and nominal length is false.
        if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed))
        {
            current->entry_speed = std::min(current->max_entry_speed, maxAllowableSpeed(-current->acceleration, next->entry_speed, current->distance));
        }
        else
        {
            current->entry_speed = current->max_entry_speed;
        }
        current->recalculate_flag = true;
    }
}

void TimeEstimateCalculator::reversePass()
{
    Block* block[3] = {nullptr, nullptr, nullptr};
    for(size_t n = blocks.size() - 1; int(n) >= 0; n--)
    {
        block[2]= block[1];
        block[1]= block[0];
        block[0] = &blocks[n];
        plannerReversePassKernel(block[0], block[1], block[2]);
    }
}

void TimeEstimateCalculator::plannerForwardPassKernel(Block *previous, Block *current, Block *next)
{
    (void)next;
    if (!previous)
    {
        return;
    }

    // If the previous block is an acceleration block, but it is not long enough to complete the
    // full speed change within the block, we need to adjust the entry speed accordingly. Entry
    // speeds have already been reset, maximized, and reverse planned by reverse planner.
    // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
    if (!previous->nominal_length_flag)
    {
        if (previous->entry_speed < current->entry_speed)
        {
            const Velocity entry_speed = std::min(current->entry_speed, maxAllowableSpeed(-previous->acceleration, previous->entry_speed, previous->distance));

            // Check for junction speed change
            if (current->entry_speed != entry_speed)
            {
                current->entry_speed = entry_speed;
                current->recalculate_flag = true;
            }
        }
    }
}

void TimeEstimateCalculator::forwardPass()
{
    Block* block[3] = {nullptr, nullptr, nullptr};
    for(size_t n = 0; n < blocks.size(); n++)
    {
        block[0]= block[1];
        block[1]= block[2];
        block[2] = &blocks[n];
        plannerForwardPassKernel(block[0], block[1], block[2]);
    }
    plannerForwardPassKernel(block[1], block[2], nullptr);
}

void TimeEstimateCalculator::recalculateTrapezoids()
{
    Block *current;
    Block *next = nullptr;

    for(unsigned int n=0; n<blocks.size(); n++)
    {
        current = next;
        next = &blocks[n];
        if (current)
        {
            // Recalculate if current block entry or exit junction speed has changed.
            if (current->recalculate_flag || next->recalculate_flag)
            {
                // NOTE: Entry and exit factors always > 0 by all previous logic operations.
                calculateTrapezoidForBlock(current, Ratio(current->entry_speed / current->nominal_feedrate), Ratio(next->entry_speed / current->nominal_feedrate));
                current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
            }
        }
    }
    // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
    if (next != nullptr)
    {
        calculateTrapezoidForBlock(next, Ratio(next->entry_speed / next->nominal_feedrate), Ratio(MINIMUM_PLANNER_SPEED / next->nominal_feedrate));
        next->recalculate_flag = false;
    }
}

}//namespace cura
