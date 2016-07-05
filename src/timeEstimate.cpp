#include <math.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include "timeEstimate.h"

namespace cura
{
    
#define MINIMUM_PLANNER_SPEED 0.05// (mm/sec)

template<typename T> const T square(const T& a) { return a * a; }

void TimeEstimateCalculator::setPosition(Position newPos)
{
    currentPosition = newPos;
}

void TimeEstimateCalculator::addTime(double time)
{
    extra_time += time;
}

void TimeEstimateCalculator::setAcceleration(double acc)
{
    acceleration = acc;
}

void TimeEstimateCalculator::setMaxXyJerk(double jerk)
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
static inline double max_allowable_speed(double acceleration, double target_velocity, double distance)
{
  return sqrt(target_velocity*target_velocity-2*acceleration*distance);
}

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the given acceleration:
static inline float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration)
{
    if (acceleration == 0)
        return 0.0;
    return (square(target_rate)-square(initial_rate)) / (2.0*acceleration);
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)
static inline double intersection_distance(double initial_rate, double final_rate, double acceleration, double distance) 
{
    if (acceleration == 0.0)
        return 0.0;
    return (2.0*acceleration*distance-square(initial_rate)+square(final_rate)) / (4.0*acceleration);
}

// This function gives the time it needs to accelerate from an initial speed to reach a final distance.
static inline double acceleration_time_from_distance(double initial_feedrate, double distance, double acceleration)
{
    double discriminant = square(initial_feedrate) - 2 * acceleration * -distance;
    //If discriminant is negative, we're moving in the wrong direction.
    //Making the discriminant 0 then gives the extremum of the parabola instead of the intersection.
    discriminant = std::max(0.0, discriminant);
    return (-initial_feedrate + sqrt(discriminant)) / acceleration;
}
    
// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.
void TimeEstimateCalculator::calculate_trapezoid_for_block(Block *block, double entry_factor, double exit_factor)
{
    double initial_feedrate = block->nominal_feedrate*entry_factor;
    double final_feedrate = block->nominal_feedrate*exit_factor;

    double acceleration = block->acceleration;
    double accelerate_distance = estimate_acceleration_distance(initial_feedrate, block->nominal_feedrate, acceleration);
    double decelerate_distance = estimate_acceleration_distance(block->nominal_feedrate, final_feedrate, -acceleration);

    // Calculate the size of Plateau of Nominal Rate.
    double plateau_distance = block->distance-accelerate_distance - decelerate_distance;

    // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
    // have to use intersection_distance() to calculate when to abort acceleration and start braking
    // in order to reach the final_rate exactly at the end of this block.
    if (plateau_distance < 0)
    {
        accelerate_distance = intersection_distance(initial_feedrate, final_feedrate, acceleration, block->distance);
        accelerate_distance = std::max(accelerate_distance, 0.0); // Check limits due to numerical round-off
        accelerate_distance = std::min(accelerate_distance, block->distance);//(We can cast here to unsigned, because the above line ensures that we are above zero)
        plateau_distance = 0;
    }

    block->accelerate_until = accelerate_distance;
    block->decelerate_after = accelerate_distance+plateau_distance;
    block->initial_feedrate = initial_feedrate;
    block->final_feedrate = final_feedrate;
}                    

void TimeEstimateCalculator::plan(Position newPos, double feedrate)
{
    Block block;
    memset(&block, 0, sizeof(block));
    
    block.maxTravel = 0;
    for(unsigned int n=0; n<NUM_AXIS; n++)
    {
        block.delta[n] = newPos[n] - currentPosition[n];
        block.absDelta[n] = fabs(block.delta[n]);
        block.maxTravel = std::max(block.maxTravel, block.absDelta[n]);
    }
    if (block.maxTravel <= 0)
        return;
    if (feedrate < minimumfeedrate)
        feedrate = minimumfeedrate;
    block.distance = sqrtf(square(block.absDelta[0]) + square(block.absDelta[1]) + square(block.absDelta[2]));
    if (block.distance == 0.0)
        block.distance = block.absDelta[3];
    block.nominal_feedrate = feedrate;
    
    Position current_feedrate;
    Position current_abs_feedrate;
    double feedrate_factor = 1.0;
    for(unsigned int n=0; n<NUM_AXIS; n++)
    {
        current_feedrate[n] = block.delta[n] * feedrate / block.distance;
        current_abs_feedrate[n] = fabs(current_feedrate[n]);
        if (current_abs_feedrate[n] > max_feedrate[n])
            feedrate_factor = std::min(feedrate_factor, max_feedrate[n] / current_abs_feedrate[n]);
    }
    //TODO: XY_FREQUENCY_LIMIT
    
    if(feedrate_factor < 1.0)
    {
        for(unsigned int n=0; n<NUM_AXIS; n++)
        {
            current_feedrate[n] *= feedrate_factor;
            current_abs_feedrate[n] *= feedrate_factor;
        }
        block.nominal_feedrate *= feedrate_factor;
    }
    
    block.acceleration = acceleration;
    for(unsigned int n=0; n<NUM_AXIS; n++)
    {
        if (block.acceleration * (block.absDelta[n] / block.distance) > max_acceleration[n])
            block.acceleration = max_acceleration[n];
    }
    
    double vmax_junction = max_xy_jerk/2; 
    double vmax_junction_factor = 1.0; 
    if(current_abs_feedrate[Z_AXIS] > max_z_jerk/2)
        vmax_junction = std::min(vmax_junction, max_z_jerk/2);
    if(current_abs_feedrate[E_AXIS] > max_e_jerk/2)
        vmax_junction = std::min(vmax_junction, max_e_jerk/2);
    vmax_junction = std::min(vmax_junction, block.nominal_feedrate);
    double safe_speed = vmax_junction;
    
    if ((blocks.size() > 0) && (previous_nominal_feedrate > 0.0001))
    {
        double xy_jerk = sqrt(square(current_feedrate[X_AXIS]-previous_feedrate[X_AXIS])+square(current_feedrate[Y_AXIS]-previous_feedrate[Y_AXIS]));
        vmax_junction = block.nominal_feedrate;
        if (xy_jerk > max_xy_jerk) {
            vmax_junction_factor = (max_xy_jerk/xy_jerk);
        } 
        if(fabs(current_feedrate[Z_AXIS] - previous_feedrate[Z_AXIS]) > max_z_jerk) {
            vmax_junction_factor = std::min(vmax_junction_factor, (max_z_jerk/fabs(current_feedrate[Z_AXIS] - previous_feedrate[Z_AXIS])));
        } 
        if(fabs(current_feedrate[E_AXIS] - previous_feedrate[E_AXIS]) > max_e_jerk) {
            vmax_junction_factor = std::min(vmax_junction_factor, (max_e_jerk/fabs(current_feedrate[E_AXIS] - previous_feedrate[E_AXIS])));
        } 
        vmax_junction = std::min(previous_nominal_feedrate, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed
    }
    
    block.max_entry_speed = vmax_junction;

    double v_allowable = max_allowable_speed(-block.acceleration, MINIMUM_PLANNER_SPEED, block.distance);
    block.entry_speed = std::min(vmax_junction, v_allowable);
    block.nominal_length_flag = block.nominal_feedrate <= v_allowable;
    block.recalculate_flag = true; // Always calculate trapezoid for new block

    previous_feedrate = current_feedrate;
    previous_nominal_feedrate = block.nominal_feedrate;

    currentPosition = newPos;

    calculate_trapezoid_for_block(&block, block.entry_speed/block.nominal_feedrate, safe_speed/block.nominal_feedrate);
    
    blocks.push_back(block);
}

double TimeEstimateCalculator::calculate()
{
    reverse_pass();
    forward_pass();
    recalculate_trapezoids();
    
    double totalTime = extra_time;
    for(unsigned int n=0; n<blocks.size(); n++)
    {
        Block& block = blocks[n];
        double plateau_distance = block.decelerate_after - block.accelerate_until;
        
        totalTime += acceleration_time_from_distance(block.initial_feedrate, block.accelerate_until, block.acceleration);
        totalTime += plateau_distance / block.nominal_feedrate;
        totalTime += acceleration_time_from_distance(block.final_feedrate, (block.distance - block.decelerate_after), block.acceleration);
    }
    return totalTime;
}

// The kernel called by accelerationPlanner::calculate() when scanning the plan from last to first entry.
void TimeEstimateCalculator::planner_reverse_pass_kernel(Block *previous, Block *current, Block *next)
{
    (void)previous;
    if(!current || !next)
        return;

    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (current->entry_speed != current->max_entry_speed)
    {
        // If nominal length true, max junction speed is guaranteed to be reached. Only compute
        // for max allowable speed if block is decelerating and nominal length is false.
        if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed))
        {
            current->entry_speed = std::min(current->max_entry_speed, max_allowable_speed(-current->acceleration, next->entry_speed, current->distance));
        } else {
            current->entry_speed = current->max_entry_speed;
        }
        current->recalculate_flag = true;
    }
}

void TimeEstimateCalculator::reverse_pass()
{
    Block* block[3] = {nullptr, nullptr, nullptr};
    for(unsigned int n=blocks.size()-1; int(n)>=0; n--)
    {
        block[2]= block[1];
        block[1]= block[0];
        block[0] = &blocks[n];
        planner_reverse_pass_kernel(block[0], block[1], block[2]);
    }
}

// The kernel called by accelerationPlanner::calculate() when scanning the plan from first to last entry.
void TimeEstimateCalculator::planner_forward_pass_kernel(Block *previous, Block *current, Block *next)
{
    (void)next;
    if(!previous)
        return;

    // If the previous block is an acceleration block, but it is not long enough to complete the
    // full speed change within the block, we need to adjust the entry speed accordingly. Entry
    // speeds have already been reset, maximized, and reverse planned by reverse planner.
    // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
    if (!previous->nominal_length_flag)
    {
        if (previous->entry_speed < current->entry_speed)
        {
            double entry_speed = std::min(current->entry_speed, max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->distance) );

            // Check for junction speed change
            if (current->entry_speed != entry_speed)
            {
                current->entry_speed = entry_speed;
                current->recalculate_flag = true;
            }
        }
    }
}

void TimeEstimateCalculator::forward_pass()
{
    Block* block[3] = {nullptr, nullptr, nullptr};
    for(unsigned int n=0; n<blocks.size(); n++)
    {
        block[0]= block[1];
        block[1]= block[2];
        block[2] = &blocks[n];
        planner_forward_pass_kernel(block[0], block[1], block[2]);
    }
    planner_forward_pass_kernel(block[1], block[2], nullptr);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
// entry_factor for each junction. Must be called by planner_recalculate() after 
// updating the blocks.
void TimeEstimateCalculator::recalculate_trapezoids()
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
                calculate_trapezoid_for_block(current, current->entry_speed/current->nominal_feedrate, next->entry_speed/current->nominal_feedrate);
                current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
            }
        }
    }
    // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
    if(next != nullptr)
    {
        calculate_trapezoid_for_block(next, next->entry_speed/next->nominal_feedrate, MINIMUM_PLANNER_SPEED/next->nominal_feedrate);
        next->recalculate_flag = false;
    }
}

}//namespace cura