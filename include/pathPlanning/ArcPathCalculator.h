// Copyright (c) 2023 BigRep GmbH
// Released under the terms of the AGPLv3 or higher.
// Author(s): Nadine Mikkelsen, Matthew McMillan (plaintoothpaste)

#ifndef PATH_PLANNING_ARC_PATH_CALCULATOR_H
#define PATH_PLANNING_ARC_PATH_CALCULATOR_H

#include <functional>
#include <vector>

#include "../settings/types/Velocity.h"
#include "../utils/AABB3D.h"
#include "../utils/Point2LL.h"
#include "../utils/Point3LL.h"


namespace cura
{

// The value to which vectors will be normalized (because the vectors/points can only be normalized to integer value lengths)
// Since usually the engine is calculating in micros this was set to one millimeter
constexpr coord_t normalization_value = 1000;

/*!
 * Class to generate an arc path (possibly a spiral with multiple turns)
 * where the beginning and the end are tangential to the previous movement done and
 * the following travel toward the next planned position. While the radius is always given
 * the turns of the spiral are calculated and added as required to keep a constant xy speed
 * and to not move faster in z direction then the provided z speed threshold.
 * Calculations are done for a discretized arc (depending on the given step size for the linear segments).
 * For each gradual z-hop one arc path is used.
 */
class ArcPath
{
public:
    const Point2LL start_; //!< The beginning of the arc
    const Point2LL end_; //!< The end point of the arc
    const coord_t radius_; //!< the radius of the underlying circle
    const Point2LL circle_center_; //!< The center of the underlying circle
    const coord_t z_increase_; //!< The z difference from the beginning to the end of the arc
    const Velocity xy_speed_; //!< The speed along the xy-plane, with which the arc should be traveled
    const Velocity z_speed_; //!< The speed along the z-axis, with which the arc should be traveled
    const coord_t discretization_step_size_; //!< The discretization step size
    const coord_t n_discrete_steps_; //!< The amount of discrete steps (at least) required for arc discretization
    const bool is_clockwise_; //!< Whether the arc is spanned clockwise or counter clockwise
    const int n_turns_; //!< The number of turns to be taken, one turn is only the arc going from start to end and higher values mean, that full turns are required as well, going
                        //!< from start to start

    /*
     * \brief Factory method, which creates an arc path where the start is tangential to the previous movement done and the end leaves the arc tangential towards a provided target
     * position.
     *
     * \param previous The previous position from which the print head is coming from
     * \param current The position at which the print head is right now
     * \param target The target position to which the print head should go
     * \param hop_height The distance in z direction by which the print head should be raised
     * \param radius The desired radius of the arc
     * \param xy_speed The speed in xy-direction with which the print head was traveling from the previous position to the current one
     * \param z_speed_limit A threshold value in how fast the print head should maximally be raised in z direction
     * \param discretization_step_size The xy-distance of one step for approximating the arc linearly (used for calculating the length of the arc when determining the speed in z
     * direction) \return The arc path object
     */
    static ArcPath calculate(
        const Point2LL previous,
        const Point2LL current,
        const Point2LL target,
        const coord_t hop_height,
        const coord_t radius,
        const Velocity xy_speed,
        const Velocity z_speed_limit,
        const coord_t discretization_step_size);

    /*
     * \brief Checks if the arc is inside some bounding box
     *
     * \param bounding_box The bounding to consider
     * \return Evaluates true, if the arc leaves the bounding box
     */
    bool isOutOfBounds(const AABB3D& bounding_box) const;

    /*
     * \brief Discretizes an arc given the xy-step size provided when it is constructed  and computes the velocity with which each point on the arc should be approached
     *
     * \param z_start The current z height
     * \return Vector of pairs including a 3d position and a velocity for approaching it. The first value is not the arc start, because this is assumed to be the current position.
     */
    std::vector<std::pair<Point3LL, Velocity>> getDiscreteArc(const coord_t z_start) const;

    ArcPath() = delete;

private:
    /*
    * \brief The constructor for an arc path, which is private because the factory method is the intended way to create this object.
    *
    * \param start_ The beginning of the arc
    * \param end_ The end point of the arc
    * \param radius_ The radius of the underlying circle
    * \param circle_center_ The center of the underlying circle
    * \param z_increase_ The z difference from the beginning to the end of the arc
    * \param xy_speed_ The speed along the xy-plane, with which the arc should be traveled
    * \param z_speed_ The speed along the z-axis, with which the arc should be traveled
    * \param step_size The discretization step size
    * \param n_discrete_steps_ The amount of discrete steps (at least) required for arc discretization
    * \param is_clockwise_ Whether the arc is spanned clockwise or counter clockwise
    * \param n_turns_ The number of turns to be taken, one turn is only the arc going from start to end and higher values mean, that full turns are required as well, going from
    start to start
    */
    ArcPath(
        const Point2LL start,
        const Point2LL end,
        const coord_t radius,
        const Point2LL circle_center,
        const coord_t z_increase,
        const Velocity xy_speed,
        const Velocity z_speed,
        const coord_t discretization_step_size,
        const coord_t n_discrete_steps,
        const bool is_clockwise,
        const int n_turns);

    /*!
     * \brief Converts an angle to a range of 0 to 2pi.
     *
     * \param angle Value to be converted
     * \return Angle inside [0,2pi]
     */
    static float convertToLimitedAngle(const float angle);

    /*!
     * \brief Calculates the x component of the coordinate of a specific tangent (depending on the provided function).
     * The tangent is calculated for a given target point outside the circle.
     *
     * \param r Circle radius
     * \param d Distance between the circle center and the target point
     * \param x X component of the target point
     * \param y Y component of the target point
     * \param func Can be subtraction or sum, depending on which tangent, on which side of the angle, should be computed. Use the sum to get the right tangent (viewed from the
     * target point) and vice versa. \return Resulting x-component of the tangent
     */
    static coord_t calcTangentX(const coord_t r, const coord_t d, const coord_t x, const coord_t y, const std::function<coord_t(coord_t, coord_t)> func);

    /*!
     * \brief Calculates the y-component of the coordinate of a specific tangent (depending on the provided function).
     * The tangent is calculated for a given target point outside the circle.
     *
     * \param r Circle radius
     * \param d Distance between the circle center and the target point
     * \param x X component of the target point
     * \param y Y component of the target point
     * \param func Can be subtraction or sum, depending on which tangent, on which side of the angle, should be computed. Use the sum to get the right tangent (viewed from the
     * target point) and vice versa. \return Resulting y-component of the tangent
     */
    static coord_t calcTangentY(const coord_t r, const coord_t d, const coord_t x, const coord_t y, std::function<float(float, float)> func);

    /*
     * \brief Calculate the angle of an arc.
     *
     * \param arc_start The starting point of the arc
     * \param arc_end The end point of the arc
     * \param circle_center The center of the circle on which the arc is based
     * \param is_clockwise Whether or not the arc is turning clockwise or counter clockwise.
     * \return The angle of the spanned arc in radians.
     */
    static float calcArcAngle(const Point2LL arc_start, const Point2LL arc_end, const Point2LL circle_center, const bool is_clockwise);

    /*
     * \brief Calculates the number of steps that have to be (at least) done to discretize the arc with a given step size.
     *
     * \param arc_start The starting point of the arc
     * \param arc_end The end point of the arc
     * \param radius The radius of the underlying circle
     * \param step_size The size of the linear discretization steps
     * \param circle_center The center of the underlying circle
     * \param is_clockwise If the arc is spanned clockwise or count clockwise
     * \return The minimal number of steps required for arc discretization
     */
    static int
        calcNumberOfSteps(const Point2LL arc_start, const Point2LL arc_end, const coord_t radius, const coord_t step_size, const Point2LL circle_center, const bool is_clockwise);
};
} // namespace cura

#endif // PATH_PLANNING_ARC_PATH_CALCULATOR_H