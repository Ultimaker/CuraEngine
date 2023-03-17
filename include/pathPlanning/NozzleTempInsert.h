// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATH_PLANNING_NOZZLE_TEMP_INSERT_H
#define PATH_PLANNING_NOZZLE_TEMP_INSERT_H

#include <compare>
#include <cstddef>

namespace cura
{

class GCodeExport;

/*!
 * A gcode command to insert before a specific path.
 *
 * Currently only used for preheat commands
 */
struct NozzleTempInsert
{
    size_t path_idx{}; //!< The path before which to insert this command
    size_t extruder{}; //!< The extruder for which to set the temp
    double temperature{}; //!< The temperature of the temperature command to insert
    bool wait{}; //!< Whether to wait for the temperature to be reached
    double time_after_path_start{ 0.0 }; //!< The time after the start of the path, before which to insert the command

    constexpr std::partial_ordering operator<=>(const NozzleTempInsert& other) const
    {
        if (path_idx == other.path_idx)
        {
            return time_after_path_start <=> other.time_after_path_start;
        }
        return path_idx <=> other.path_idx;
    }

    /*!
     * Write the temperature command at the current position in the gcode.
     * \param gcode The actual gcode writer
     */
    void write(GCodeExport& gcode);

};

} // namespace cura

#endif // PATH_PLANNING_NOZZLE_TEMP_INSERT_H
