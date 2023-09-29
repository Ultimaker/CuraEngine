// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATH_PLANNING_TIME_MATERIAL_ESTIMATES_H
#define PATH_PLANNING_TIME_MATERIAL_ESTIMATES_H

namespace cura
{

struct TimeMaterialEstimates
{
    double extrude_time{ 0.0 }; //!< Time in seconds occupied by extrusion
    double unretracted_travel_time{ 0.0 }; //!< Time in seconds occupied by non-retracted travel (non-extrusion)
    double retracted_travel_time{ 0.0 }; //!< Time in seconds occupied by retracted travel (non-extrusion)
    double material{ 0.0 }; //!< Material used (in mm^3)
    double extrude_time_at_slowest_path_speed{ 0.0 }; //!< Time in seconds occupied by extrusion assuming paths are printed at slowest path speed, usually the outer wall speed
    double extrude_time_at_minimum_speed{ 0.0 }; //!< Time in seconds occupied by extrusion assuming paths are printed at the user specified Minimum Speed

    constexpr TimeMaterialEstimates& operator+=(const TimeMaterialEstimates& other) noexcept
    {
        extrude_time += other.extrude_time;
        extrude_time_at_slowest_path_speed += other.extrude_time_at_slowest_path_speed;
        extrude_time_at_minimum_speed += other.extrude_time_at_minimum_speed;
        unretracted_travel_time += other.unretracted_travel_time;
        retracted_travel_time += other.retracted_travel_time;
        material += other.material;
        return *this;
    }

    constexpr TimeMaterialEstimates& operator-=(const TimeMaterialEstimates& other) noexcept
    {
        extrude_time -= other.extrude_time;
        extrude_time_at_slowest_path_speed -= other.extrude_time_at_slowest_path_speed;
        extrude_time_at_minimum_speed -= other.extrude_time_at_minimum_speed;
        unretracted_travel_time -= other.unretracted_travel_time;
        retracted_travel_time -= other.retracted_travel_time;
        material -= other.material;
        return *this;
    }

    constexpr TimeMaterialEstimates operator+(const TimeMaterialEstimates& other) const noexcept
    {
        return { .extrude_time = extrude_time + other.extrude_time,
                 .unretracted_travel_time = unretracted_travel_time + other.unretracted_travel_time,
                 .retracted_travel_time = retracted_travel_time + other.retracted_travel_time,
                 .material = material + other.material,
                 .extrude_time_at_slowest_path_speed = extrude_time_at_slowest_path_speed + other.extrude_time_at_slowest_path_speed,
                 .extrude_time_at_minimum_speed = extrude_time_at_minimum_speed + other.extrude_time_at_minimum_speed };
    };

    constexpr TimeMaterialEstimates operator-(const TimeMaterialEstimates& other) const noexcept
    {
        return { .extrude_time = extrude_time - other.extrude_time,
                 .unretracted_travel_time = unretracted_travel_time - other.unretracted_travel_time,
                 .retracted_travel_time = retracted_travel_time - other.retracted_travel_time,
                 .material = material - other.material,
                 .extrude_time_at_slowest_path_speed = extrude_time_at_slowest_path_speed - other.extrude_time_at_slowest_path_speed,
                 .extrude_time_at_minimum_speed = extrude_time_at_minimum_speed - other.extrude_time_at_minimum_speed };
    };

    constexpr auto operator<=>(const TimeMaterialEstimates& other) const noexcept = default;

    constexpr void reset() noexcept
    {
        *this = TimeMaterialEstimates{};
    }

    [[nodiscard]] constexpr auto getTotalTime() const noexcept
    {
        return extrude_time + unretracted_travel_time + retracted_travel_time;
    }

    [[nodiscard]] constexpr auto getTotalUnretractedTime() const noexcept
    {
        return extrude_time + unretracted_travel_time;
    }

    [[nodiscard]] constexpr auto getTravelTime() const noexcept
    {
        return retracted_travel_time + unretracted_travel_time;
    }
};

} // namespace cura

#endif // PATH_PLANNING_TIME_MATERIAL_ESTIMATES_H
