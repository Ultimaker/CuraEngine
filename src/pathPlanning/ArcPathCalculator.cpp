// Copyright (c) 2023 BigRep GmbH

#include "pathPlanning/ArcPathCalculator.h"

namespace cura
{

ArcPath ArcPath::calculate(
    const Point2LL previous,
    const Point2LL current,
    const Point2LL target,
    const coord_t hop_height,
    const coord_t radius,
    const Velocity xy_speed,
    const Velocity z_speed_limit,
    const coord_t step_size)
{
    assert(radius != 0 && "Arc radius should be larger then zero.");
    const Point2LL last_direction = current - previous;
    const Point2LL center_vec = normal(last_direction, radius);
    auto orthogonal_last_vec = Point2LL{ -last_direction.Y, last_direction.X };
    orthogonal_last_vec
        = normal(orthogonal_last_vec, normalization_value); // normalization to length 1 results in (0,0) since it is a ClipperLib::IntPoint, normalize to 1 mm hence 1000 um
    const Point2LL target_previus_vec = normal(target - previous, normalization_value);
    // When calculating the dot product of vectors which are not normalized keep in mind, that it involves squaring the magnitude of the dot product
    // Check if it is parallel and inverse direction
    const bool inverse_direction = dot(normal(last_direction, normalization_value), normal(target - current, normalization_value)) == -(normalization_value * normalization_value);
    const float dot_product = dot(orthogonal_last_vec, target_previus_vec) / static_cast<float>(normalization_value * normalization_value);

    // Step 1: Determine the nature of the underlying circle:
    // On which side of the previous movement the circle should be located and
    // in which direction should we travel the arc (clockwise?) and
    // which (of two possible) tangents should be used as exit point from the arc
    std::function<coord_t(coord_t, coord_t)> tangent_func;
    bool clockwise_rotation;
    Point2LL circle_center;
    if ((dot_product == 0) && ! inverse_direction) // colinear edge case
    {
        clockwise_rotation = true;
        circle_center = Point2LL{ center_vec.Y, -center_vec.X };
    }
    else if (dot_product > 0) // circle is on the left side
    {
        clockwise_rotation = false;
        circle_center = Point2LL{ -center_vec.Y, center_vec.X };
        tangent_func = [](coord_t a, coord_t b) -> coord_t
        {
            return a - b;
        };
    }
    else // circle is on the right side
    {
        clockwise_rotation = true;
        circle_center = Point2LL{ center_vec.Y, -center_vec.X };
        tangent_func = [](coord_t a, coord_t b) -> coord_t
        {
            return a + b;
        };
    }
    circle_center = circle_center + current;

    // Step 2: Calculate the tangent point at which we should leave the arc
    const coord_t dist_to_target = vSize(circle_center - target);
    Point2LL arc_end;
    if (dist_to_target == radius) // edge case: the target position is located exactly on the circle
    {
        arc_end = target;
    }
    else if (((dot_product == 0) && ! inverse_direction) || dist_to_target < radius) // edge case: points are colinear or the target point is inside the circle
    {
        arc_end = current;
    }
    else
    {
        const Point2LL centered_target = target - circle_center;
        const coord_t tangent_x = calcTangentX(radius, dist_to_target, centered_target.X, centered_target.Y, tangent_func);
        const coord_t tangent_y = calcTangentY(radius, dist_to_target, centered_target.X, centered_target.Y, tangent_func);
        arc_end = Point2LL{ tangent_x, tangent_y } + circle_center;
    }

    // Step 3: Determine if the arc is sufficient to raise the print head in z-direction without exceeding the z-speed threshold
    // Add spiral turns when the z-speed would be too high
    // Since in many cases the arc will be linearly approximated for or by the printer board, this calculation is done by assuming linear discretization steps
    int n_discrete_steps = calcNumberOfSteps(current, arc_end, radius, step_size, circle_center, clockwise_rotation);
    float z_height_per_segment = static_cast<float>(hop_height) / n_discrete_steps;
    const int n_steps_full_turn = calcNumberOfSteps(current, current, radius, step_size, circle_center, clockwise_rotation);
    int n_turns = 1;
    auto z_speed = Velocity{ 0 };
    while (true)
    {
        // Velocity is always defined in mm/sec and the distance in um (micro meter), but the unit conversion here is mathematically redundant
        z_speed = (z_height_per_segment * (xy_speed /* *1000 */ / step_size)) /* /1000  */;
        if (z_speed <= z_speed_limit) // z speed is not too high
        {
            break;
        }
        ++n_turns;
        n_discrete_steps += n_steps_full_turn;
        z_height_per_segment = static_cast<float>(hop_height) / n_discrete_steps;
    }

    return ArcPath(current, arc_end, radius, circle_center, hop_height, xy_speed, z_speed, step_size, n_discrete_steps, clockwise_rotation, n_turns);
}

bool ArcPath::isOutOfBounds(const AABB3D& bounding_box) const
{
    return circle_center.Y - radius <= bounding_box.min_.y_ || circle_center.X - radius <= bounding_box.min_.x_ || circle_center.Y + radius >= bounding_box.max_.y_
        || circle_center.X + radius >= bounding_box.max_.x_;
}

std::vector<std::pair<Point3LL, Velocity>> ArcPath::getDiscreteArc(const coord_t z_start) const
{
    const coord_t z_end = z_start + z_increase;
    assert(z_end >= z_start && "The start height of a z-hop needs to be less then the end height.");
    // Calculate the angle difference on the circle between two different steps of the discretization
    const Point2LL centered_start = normal(start - circle_center, normalization_value);
    const float angle_to_start
        = convertToLimitedAngle(std::atan2(centered_start.Y / static_cast<float>(normalization_value), centered_start.X / static_cast<float>(normalization_value)));
    const float arc_angle = calcArcAngle(start, end, circle_center, is_clockwise);
    const float angle_per_step = (arc_angle + (n_turns - 1) * 2 * std::numbers::pi) / n_discrete_steps;
    const float height_per_step = static_cast<float>(z_end - z_start) / n_discrete_steps;
    const Velocity vel = sqrt(xy_speed * xy_speed + z_speed * z_speed);

    // This vector is not including the current position, which is the start point of the arc
    std::vector<std::pair<Point3LL, Velocity>> arc_points{};
    arc_points.reserve(n_discrete_steps);
    // For each discretization rotate the starting position by an increasing angle and keep a constant velocity for each step
    for (size_t i = 1; i < n_discrete_steps; ++i)
    {
        const auto angle = is_clockwise ? convertToLimitedAngle(angle_to_start - i * angle_per_step) : convertToLimitedAngle(angle_to_start + i * angle_per_step);
        auto pt = Point3LL{ static_cast<coord_t>(cos(angle) * radius + circle_center.X),
                            static_cast<coord_t>(sin(angle) * radius + circle_center.Y),
                            static_cast<coord_t>(z_start + i * height_per_step) };
        arc_points.emplace_back(pt, vel);
    }
    arc_points.emplace_back(Point3LL{ end.X, end.Y, z_end }, vel);
    return arc_points;
}

ArcPath::ArcPath(
    const Point2LL start_,
    const Point2LL end_,
    const coord_t radius_,
    const Point2LL circle_center_,
    const coord_t z_increase_,
    const Velocity xy_speed_,
    const Velocity z_speed_,
    const coord_t step_size_,
    const coord_t n_discrete_steps_,
    const bool is_clockwise_,
    const int n_turns_)
    : start(start_)
    , end(end_)
    , radius(radius_)
    , circle_center(circle_center_)
    , z_increase(z_increase_)
    , xy_speed(xy_speed_)
    , z_speed(z_speed_)
    , step_size(step_size_)
    , n_discrete_steps(n_discrete_steps_)
    , is_clockwise(is_clockwise_)
    , n_turns(n_turns_)
{
}

float ArcPath::convertToLimitedAngle(const float angle)
{
    float properAngle = angle;
    while (properAngle < 0)
    {
        properAngle = 2 * std::numbers::pi - abs(properAngle);
    }
    while (properAngle > 2 * static_cast<float>(std::numbers::pi))
    {
        properAngle = properAngle - 2 * std::numbers::pi;
    }
    return properAngle;
}


coord_t ArcPath::calcTangentX(const coord_t r, const coord_t d, const coord_t x, const coord_t y, const std::function<coord_t(coord_t, coord_t)> func)
{
    // for the tangent calculation see: https://en.wikipedia.org/wiki/Tangent_lines_to_circles
    const float d_2 = d * d;
    const float r_2 = r * r;
    return func(x * (r_2 / d_2), (r / d_2) * sqrt(d_2 - r_2) * (-y));
}

coord_t ArcPath::calcTangentY(const coord_t r, const coord_t d, const coord_t x, const coord_t y, const std::function<float(float, float)> func)
{
    // for the tangent calculation see: https://en.wikipedia.org/wiki/Tangent_lines_to_circles
    const float d_2 = d * d;
    const float r_2 = r * r;
    return func(y * (r_2 / d_2), ((r * x) / d_2) * sqrt(d_2 - r_2));
}

float ArcPath::calcArcAngle(const Point2LL arc_start, const Point2LL arc_end, const Point2LL circle_center, const bool is_clockwise)
{
    // if the arc start and end point coincide it should be full circle instead of a zero length arc
    if (arc_start == arc_end)
    {
        return 2.0 * std::numbers::pi;
    }
    const Point2LL centered_start = normal(arc_start - circle_center, normalization_value);
    const Point2LL centered_end = normal(arc_end - circle_center, normalization_value);
    // atan2 range is normally in the interval of [-pi,pi] for further calculations shift the range to [0,2*pi] to always calculate with positive angles
    const float angle_to_start
        = convertToLimitedAngle(std::atan2(centered_start.Y / static_cast<float>(normalization_value), centered_start.X / static_cast<float>(normalization_value)));
    const float angle_to_end
        = convertToLimitedAngle(std::atan2(centered_end.Y / static_cast<float>(normalization_value), centered_end.X / static_cast<float>(normalization_value)));
    return convertToLimitedAngle(is_clockwise ? angle_to_start - angle_to_end : angle_to_end - angle_to_start);
}

int ArcPath::calcNumberOfSteps(
    const Point2LL arc_start,
    const Point2LL arc_end,
    const coord_t radius,
    const coord_t step_size,
    const Point2LL circle_center,
    const bool is_clockwise)
{
    const float arc_angle = calcArcAngle(arc_start, arc_end, circle_center, is_clockwise);
    const float step_angle = std::acos(1 - pow(step_size, 2) / (2 * pow(radius, 2)));
    const int n_steps = floor(arc_angle / step_angle);
    return n_steps > 0 ? n_steps : 1;
}

} // namespace cura