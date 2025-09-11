// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef ENUMSETTINGS_H
#define ENUMSETTINGS_H

namespace cura
{
/*!
 * In Cura different infill methods are available.
 * This enum defines which fill patterns are available to get a uniform naming throughout the engine.
 * The different methods are used for top/bottom, support and sparse infill.
 */
enum class EFillMethod
{
    LINES,
    GRID,
    CUBIC,
    CUBICSUBDIV,
    TETRAHEDRAL,
    QUARTER_CUBIC,
    TRIANGLES,
    TRIHEXAGON,
    CONCENTRIC,
    ZIG_ZAG,
    CROSS,
    CROSS_3D,
    GYROID,
    LIGHTNING,
    NONE, // NOTE: Should remain second last! Before PLUGIN (Might be used in testing to enumerate the enum.)
    PLUGIN, // Place plugin after none to prevent it from being tested in the gtest suite.
};


/*!
 * Enum for the value of extra_infill_lines_to_support_skins
 * This enum defines what extra lines should be added to infill to support
 * skins above.
 */
enum class EExtraInfillLinesToSupportSkins
{
    WALLS_AND_LINES,
    WALLS,
    NONE,
};

/*!
 * Type of platform adhesion.
 */
enum class EPlatformAdhesion
{
    SKIRT,
    BRIM,
    RAFT,
    NONE,
    PLUGIN,
};

/*!
 * Type of support material to generate
 */
enum class ESupportType
{
    NONE,
    PLATFORM_ONLY,
    EVERYWHERE,
    PLUGIN,
};

/*!
 * Structure of the support, such as Tree Support
 */
enum class ESupportStructure
{
    NORMAL,
    TREE,
    PLUGIN,
};

enum class EZSeamType
{
    RANDOM,
    SHORTEST,
    USER_SPECIFIED,
    SHARPEST_CORNER,

    /* The 'Skirt/brim' type behaves like shortest, except it doesn't try to do tie-breaking for similar locations to
     * the last attempt, as that gives a different result when the seams are next to each other instead of on top.
     */
    SKIRT_BRIM,
    PLUGIN,
};

enum class EZSeamCornerPrefType
{
    Z_SEAM_CORNER_PREF_INNER,
    Z_SEAM_CORNER_PREF_OUTER,
    Z_SEAM_CORNER_PREF_ANY,
    Z_SEAM_CORNER_PREF_WEIGHTED,
    PLUGIN,
};

enum class ESurfaceMode
{
    NORMAL,
    SURFACE,
    BOTH,
    PLUGIN,
};

enum class FillPerimeterGapMode
{
    NOWHERE,
    EVERYWHERE,
    PLUGIN,
};

enum class BuildPlateShape
{
    RECTANGULAR,
    ELLIPTIC,
    PLUGIN,
};

enum class CombingMode
{
    OFF,
    ALL,
    NO_SKIN,
    NO_OUTER_SURFACES,
    INFILL,
    PLUGIN,
};

/*!
 * How the draft shield height is limited.
 */
enum class DraftShieldHeightLimitation
{
    FULL, // Draft shield takes full height of the print.
    LIMITED, // Draft shield is limited by draft_shield_height setting.
    PLUGIN,
};

enum class SupportDistPriority
{
    XY_OVERRIDES_Z,
    Z_OVERRIDES_XY,
    PLUGIN,
};

enum class SlicingTolerance
{
    MIDDLE,
    INCLUSIVE,
    EXCLUSIVE,
    PLUGIN,
};
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

/*!
 * Direction in which to print walls, inside vs. outside.
 */
enum class InsetDirection
{
    /*!
     * The innermost wall is printed first, then the second-innermost wall, etc.
     */
    INSIDE_OUT,

    /*!
     * The outermost wall is printed first, then the second wall, etc.
     */
    OUTSIDE_IN,

    /*!
     * If the innermost wall is a central wall, it is printed last. Otherwise
     * prints the same as inside out.
     */
    CENTER_LAST,
    PLUGIN,
};

/*!
 * Prime tower generation mode
 */
enum class PrimeTowerMode
{
    /*!
     * Prime tower that minimizes time and used filament as much as possible.
     */
    INTERLEAVED,

    /*!
     * Prime tower that minimizes time and used filament, but doesn't allow
     * for printing two different filaments over each other.
     */
    NORMAL,
};

/*!
 * Brim location, inside, outside or both
 */
enum class BrimLocation
{
    OUTSIDE = 0x01, // Brim only on the outside of the model
    INSIDE = 0x02, // Brim only on the inside of the model
    EVERYWHERE = 0x03, // Brim on both the outside and inside of the model
};

/*!
 * How to enable/disable cooling fan(s) during extruder switch
 */
enum class CoolDuringExtruderSwitch
{
    UNCHANGED, // Let fans as they are during nozzle switch (historical behavior)
    ONLY_LAST_EXTRUDER, // Turn on fan of the previously used extruder to cool it down, turn others off
    ALL_FANS, // Turn on all fans
};

/*!
 * Convenience binary operator to allow testing brim location easily, like (actual_location & BrimLocation::OUTSIDE)
 */
[[maybe_unused]] static int operator&(BrimLocation location1, BrimLocation location2)
{
    return static_cast<int>(location1) & static_cast<int>(location2);
}

} // namespace cura

#endif // ENUMSETTINGS_H
