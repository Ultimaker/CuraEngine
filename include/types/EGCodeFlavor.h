// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace curaengine::types
{
/*!
 * Different flavors of GCode. Some machines require different types of GCode.
 * The GCode flavor definition handles this as a big setting to make major or minor modifications to the GCode.
 */
enum class EGCodeFlavor
{
    /**
     * Marlin flavored GCode is Marlin/Sprinter based GCode.
     *  This is the most commonly used GCode set.
     *  G0 for moves, G1 for extrusion.
     *  E values give mm of filament extrusion.
     *  Retraction is done on E values with G1. Start/end code is added.
     *  M106 Sxxx and M107 are used to turn the fan on/off.
     **/
    MARLIN = 0,
    /**
     * UltiGCode flavored is Marlin based GCode.
     *  UltiGCode uses less settings on the slicer and puts more settings in the firmware. This makes for more hardware/material independed GCode.
     *  G0 for moves, G1 for extrusion.
     *  E values give mm^3 of filament extrusion. Ignores the filament diameter setting.
     *  Retraction is done with G10 and G11. Retraction settings are ignored. G10 S1 is used for multi-extruder switch retraction.
     *  Start/end code is not added.
     *  M106 Sxxx and M107 are used to turn the fan on/off.
     **/
    ULTIGCODE = 1,
    /**
     * Makerbot flavored GCode.
     *  Looks a lot like RepRap GCode with a few changes. Requires MakerWare to convert to X3G files.
     *   Heating needs to be done with M104 Sxxx T0
     *   No G21 or G90
     *   Fan ON is M126 T0 (No fan strength control?)
     *   Fan OFF is M127 T0
     *   Homing is done with G162 X Y F2000
     **/
    MAKERBOT = 2,

    /**
     * Bits From Bytes GCode.
     *  BFB machines use RPM instead of E. Which is coupled to the F instead of independed. (M108 S[deciRPM])
     *  Need X,Y,Z,F on every line.
     *  Needs extruder ON/OFF (M101, M103), has auto-retrection (M227 S[2560*mm] P[2560*mm])
     **/
    BFB = 3,

    /**
     * MACH3 GCode
     *  MACH3 is CNC control software, which expects A/B/C/D for extruders, instead of E.
     **/
    MACH3 = 4,
    /**
     * RepRap volumatric flavored GCode is Marlin based GCode.
     *  Volumatric uses less settings on the slicer and puts more settings in the firmware. This makes for more hardware/material independed GCode.
     *  G0 for moves, G1 for extrusion.
     *  E values give mm^3 of filament extrusion. Ignores the filament diameter setting.
     *  Retraction is done with G10 and G11. Retraction settings are ignored. G10 S1 is used for multi-extruder switch retraction.
     *  M106 Sxxx and M107 are used to turn the fan on/off.
     **/
    MARLIN_VOLUMATRIC = 5,
    /**
     * Griffin flavored is Marlin based GCode.
     *  This is a type of RepRap used for machines with multiple extruder trains.
     *  G0 for moves, G1 for extrusion.
     *  E values give mm of filament extrusion.
     *  E values are stored separately per extruder train.
     *  Retraction is done on E values with G1. Start/end code is added.
     *  M227 is used to initialize a single extrusion train.
     **/
    GRIFFIN = 6,
    /**
     * Cheetah flavored is Griffin based, but with the Cheetah planner.
     *  This means it has a jerk-limited motion profile based on real jerk (instead of Marlin's jump).
     *  The jerk value is set using M215 in m/s^3
     **/
    CHEETAH = 61,

    REPETIER = 7,

    /**
     * Real RepRap GCode suitable for printers using RepRap firmware (e.g. Duet controllers)
     **/
    REPRAP = 8,
    PLUGIN = 9,
};

} // namespace curaengine::types
