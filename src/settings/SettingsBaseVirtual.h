//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SETTINGSBASEVIRTUAL_H
#define SETTINGSBASEVIRTUAL_H

#include "../FlowTempGraph.h"
#include "../utils/floatpoint.h"
#include "../utils/IntPoint.h" //For coord_t.

namespace cura
{

/*!
 * In Cura different infill methods are available.
 * This enum defines which fill patterns are available to get a uniform naming troughout the engine.
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
    NONE
};

/*!
 * Type of platform adhesion.
 */
enum class EPlatformAdhesion
{
    SKIRT,
    BRIM,
    RAFT,
    NONE
};

/*!
 * Type of support material to generate
 */
enum class ESupportType
{
    NONE,
    PLATFORM_ONLY,
    EVERYWHERE
};

enum class EZSeamType
{
    RANDOM,
    SHORTEST,
    USER_SPECIFIED,
    SHARPEST_CORNER
};

enum class EZSeamCornerPrefType
{
    Z_SEAM_CORNER_PREF_NONE,
    Z_SEAM_CORNER_PREF_INNER,
    Z_SEAM_CORNER_PREF_OUTER,
    Z_SEAM_CORNER_PREF_ANY
};

enum class ESurfaceMode
{
    NORMAL,
    SURFACE,
    BOTH
};

enum class FillPerimeterGapMode
{
    NOWHERE,
    EVERYWHERE
};

enum class BuildPlateShape
{
    RECTANGULAR,
    ELLIPTIC
};

enum class CombingMode
{
    OFF,
    ALL,
    NO_SKIN
};

/*!
 * How the draft shield height is limited.
 */
enum class DraftShieldHeightLimitation
{
    FULL, //Draft shield takes full height of the print.
    LIMITED //Draft shield is limited by draft_shield_height setting.
};

enum class SupportDistPriority
{
    XY_OVERRIDES_Z,
    Z_OVERRIDES_XY
};

enum class SlicingTolerance
{
    MIDDLE,
    INCLUSIVE,
    EXCLUSIVE
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

    REPETIER = 7,

/**
 * Real RepRap GCode suitable for printers using RepRap firmware (e.g. Duet controllers)
 **/
    REPRAP = 8,
};

/*!
 * An abstract class for classes that can provide setting values.
 * These are: SettingsBase, which contains setting information 
 * and SettingsMessenger, which can pass on setting information from a SettingsBase
 */
class SettingsBaseVirtual
{
protected:
    SettingsBaseVirtual* parent;
public:
    virtual const std::string& getSettingString(const std::string& key) const = 0;
    
    virtual void setSetting(std::string key, std::string value) = 0;

    /*!
     * Set the parent settings base for inheriting a setting to a specific setting base.
     * This overrides the use of \ref SettingsBaseVirtual::parent.
     * 
     * \param key The setting for which to override the inheritance
     * \param parent The setting base from which to obtain the setting instead.
     */
    virtual void setSettingInheritBase(std::string key, const SettingsBaseVirtual& parent) = 0;

    virtual ~SettingsBaseVirtual() {}
    
    SettingsBaseVirtual(); //!< SettingsBaseVirtual without a parent settings object
    SettingsBaseVirtual(SettingsBaseVirtual* parent); //!< construct a SettingsBaseVirtual with a parent settings object
    
    void setParent(SettingsBaseVirtual* parent) { this->parent = parent; }
    SettingsBaseVirtual* getParent() { return parent; }
    
    int getSettingAsIndex(std::string key) const;
    int getSettingAsCount(std::string key) const;

    /*!
     * Get a setting as an int, but if it's -1 then return
     * the value of the setting "extruder_nr"
     */
    int getSettingAsExtruderNr(std::string key) const;

    /*!
     * \brief Interprets a setting as a layer number.
     *
     * The input of the layer number is one-based. This translates it to
     * zero-based numbering.
     *
     * \return Zero-based numbering of a layer number setting.
     */
    unsigned int getSettingAsLayerNumber(std::string key) const;

    double getSettingInAngleDegrees(std::string key) const;
    double getSettingInAngleRadians(std::string key) const;
    double getSettingInMillimeters(std::string key) const;
    coord_t getSettingInMicrons(std::string key) const;
    bool getSettingBoolean(std::string key) const;
    double getSettingInDegreeCelsius(std::string key) const;
    double getSettingInMillimetersPerSecond(std::string key) const;
    double getSettingInCubicMillimeters(std::string key) const;
    double getSettingInPercentage(std::string key) const;
    double getSettingAsRatio(std::string key) const; //!< For settings which are provided in percentage
    double getSettingInSeconds(std::string key) const;

    FlowTempGraph getSettingAsFlowTempGraph(std::string key) const;
    FMatrix3x3 getSettingAsPointMatrix(std::string key) const;

    DraftShieldHeightLimitation getSettingAsDraftShieldHeightLimitation(const std::string key) const;
    EGCodeFlavor getSettingAsGCodeFlavor(std::string key) const;
    EFillMethod getSettingAsFillMethod(std::string key) const;
    EPlatformAdhesion getSettingAsPlatformAdhesion(std::string key) const;
    ESupportType getSettingAsSupportType(std::string key) const;
    EZSeamType getSettingAsZSeamType(std::string key) const;
    EZSeamCornerPrefType getSettingAsZSeamCornerPrefType(std::string key) const;
    ESurfaceMode getSettingAsSurfaceMode(std::string key) const;
    FillPerimeterGapMode getSettingAsFillPerimeterGapMode(std::string key) const;
    BuildPlateShape getSettingAsBuildPlateShape(const std::string& key) const;
    CombingMode getSettingAsCombingMode(std::string key) const;
    SupportDistPriority getSettingAsSupportDistPriority(std::string key) const;
    SlicingTolerance getSettingAsSlicingTolerance(std::string key) const;
    std::vector<int> getSettingAsIntegerList(std::string key) const;
};

} //Cura namespace.

#endif //SETTINGSBASEVIRTUAL_H