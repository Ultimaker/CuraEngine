// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GCODEEXPORT_H
#define GCODEEXPORT_H

#include <deque> // for extrusionAmountAtPreviousRetractions
#ifdef BUILD_TESTS
#include <gtest/gtest_prod.h> //To allow tests to use protected members.
#endif
#include <optional>
#include <sstream> // for stream.str()
#include <stdio.h>

#include "TravelAntiOozing.h"
#include "geometry/Point2LL.h"
#include "settings/EnumSettings.h"
#include "settings/Settings.h" //For MAX_EXTRUDERS.
#include "settings/types/LayerIndex.h"
#include "settings/types/Temperature.h" //Bed temperature.
#include "settings/types/Velocity.h"
#include "timeEstimate.h"
#include "utils/AABB3D.h" //To track the used build volume for the Griffin header.
#include "utils/NoCopy.h"
#include "utils/string.h"

namespace cura
{

class RetractionConfig;
class SliceDataStorage;
struct WipeScriptConfig;

// The GCodeExport class writes the actual GCode. This is the only class that knows how GCode looks and feels.
//   Any customizations on GCodes flavors are done in this class.
class GCodeExport : public NoCopy
{
#ifdef BUILD_TESTS
    friend class GCodeExportTest;
    friend class GriffinHeaderTest;
    friend class AntiOozeAmountsTest;
    FRIEND_TEST(GCodeExportTest, CommentEmpty);
    FRIEND_TEST(GCodeExportTest, CommentSimple);
    FRIEND_TEST(GCodeExportTest, CommentMultiLine);
    FRIEND_TEST(GCodeExportTest, CommentMultiple);
    FRIEND_TEST(GCodeExportTest, CommentTimeZero);
    FRIEND_TEST(GCodeExportTest, CommentTimeInteger);
    FRIEND_TEST(GCodeExportTest, CommentTimeFloatRoundingError);
    FRIEND_TEST(GCodeExportTest, CommentTypeAllTypesCovered);
    FRIEND_TEST(GCodeExportTest, CommentLayer);
    FRIEND_TEST(GCodeExportTest, CommentLayerNegative);
    FRIEND_TEST(GCodeExportTest, CommentLayerCount);
    FRIEND_TEST(GriffinHeaderTest, HeaderGriffinFormat);
    FRIEND_TEST(GCodeExportTest, HeaderUltiGCode);
    FRIEND_TEST(GCodeExportTest, HeaderRepRap);
    FRIEND_TEST(GCodeExportTest, HeaderMarlin);
    FRIEND_TEST(GCodeExportTest, HeaderMarlinVolumetric);
    FRIEND_TEST(GCodeExportTest, EVsMmVolumetric);
    FRIEND_TEST(GCodeExportTest, EVsMmLinear);
    FRIEND_TEST(GCodeExportTest, WriteZHopStartDefaultSpeed);
    FRIEND_TEST(GCodeExportTest, WriteZHopStartCustomSpeed);
    FRIEND_TEST(GCodeExportTest, WriteZHopEndZero);
    FRIEND_TEST(GCodeExportTest, WriteZHopEndDefaultSpeed);
    FRIEND_TEST(GCodeExportTest, WriteZHopEndCustomSpeed);
    FRIEND_TEST(GCodeExportTest, insertWipeScriptSingleMove);
    FRIEND_TEST(GCodeExportTest, insertWipeScriptMultipleMoves);
    FRIEND_TEST(GCodeExportTest, insertWipeScriptOptionalDelay);
    FRIEND_TEST(GCodeExportTest, insertWipeScriptRetractionEnable);
    FRIEND_TEST(GCodeExportTest, insertWipeScriptHopEnable);
#endif
private:
    struct ExtruderTrainAttributes
    {
        bool is_primed_; //!< Whether this extruder has currently already been primed in this print

        bool is_used_; //!< Whether this extruder train is actually used during the printing of all meshgroups
        char extruder_character_;

        double filament_area_; //!< in mm^2 for non-volumetric, cylindrical filament

        double total_filament_; //!< total filament used per extruder in mm^3
        Temperature current_temperature_;
        bool waited_for_temperature_; //!< Whether the most recent temperature command has been a heat-and-wait command (M109) or not (M104).
        Temperature initial_temp_; //!< Temperature this nozzle needs to be at the start of the print.

        double retraction_e_amount_current_; //!< The current retracted amount (in mm or mm^3), or zero(i.e. false) if it is not currently retracted (positive values mean retracted
                                             //!< amount, so negative impact on E values)
        double retraction_e_amount_at_e_start_; //!< The ExtruderTrainAttributes::retraction_amount_current value at E0, i.e. the offset (in mm or mm^3) from E0 to the situation
                                                //!< where the filament is at the tip of the nozzle.

        double prime_volume_; //!< Amount of material (in mm^3) to be primed after an unretration (due to oozing and/or coasting)
        Velocity last_retraction_prime_speed_; //!< The last prime speed (in mm/s) of the to-be-primed amount

        double last_e_value_after_wipe_; //!< The current material amount extruded since last wipe

        size_t fan_number_; // nozzle print cooling fan number
        Point2LL nozzle_offset_; //!< Cache of setting machine_nozzle_offset_[xy]
        bool machine_firmware_retract_; //!< Cache of setting machine_firmware_retract

        std::deque<double> extruded_volume_at_previous_n_retractions_; // in mm^3

        ExtruderTrainAttributes()
            : is_primed_(false)
            , is_used_(false)
            , extruder_character_(0)
            , filament_area_(0)
            , total_filament_(0)
            , current_temperature_(0)
            , waited_for_temperature_(false)
            , initial_temp_(0)
            , retraction_e_amount_current_(0.0)
            , retraction_e_amount_at_e_start_(0.0)
            , prime_volume_(0.0)
            , last_retraction_prime_speed_(0.0)
            , fan_number_(0)
        {
        }
    };

    struct RetractionAmounts
    {
        double old_e{ 0.0 }; // The previous absolute retraction amount
        double new_e{ 0.0 }; // The new absolute retraction amount
        double diff_e{ 0.0 }; // The difference between the new and previous amount, which is to be processed. Positive means retraction.

        /*!
         * Indicates whether this retraction actually has something to process
         */
        [[nodiscard]] bool has_retraction() const noexcept
        {
            constexpr double threshold{ 1e-6 };
            return std::abs(diff_e) >= threshold;
        }
    };

    ExtruderTrainAttributes extruder_attr_[MAX_EXTRUDERS];
    bool use_extruder_offset_to_offset_coords_;
    std::string machine_name_;
    std::string slice_uuid_; //!< The UUID of the current slice.

    std::ostream* output_stream_;
    std::string new_line_;

    double current_e_value_; //!< The last E value written to gcode (in mm or mm^3)

    // flow-rate compensation
    double current_e_offset_; //!< Offset to compensate for flow rate (mm or mm^3)
    double max_extrusion_offset_; //!< 0 to turn it off, normally 4
    double extrusion_offset_factor_; //!< default 1

    Point3LL current_position_; //!< The last build plate coordinates written to gcode (which might be different from actually written gcode coordinates when the extruder offset is
                                //!< encoded in the gcode)
    Velocity current_speed_; //!< The current speed (F values / 60) in mm/s
    Acceleration current_print_acceleration_; //!< The current acceleration (in mm/s^2) used for print moves (and also for travel moves if the gcode flavor doesn't have separate
                                              //!< travel acceleration)
    Acceleration
        current_travel_acceleration_; //!< The current acceleration (in mm/s^2) used for travel moves for those gcode flavors that have separate print and travel accelerations
    Velocity current_jerk_; //!< The current jerk in the XY direction (in mm/s^3)

    AABB3D total_bounding_box_; //!< The bounding box of all g-code.

    /*!
     * The z position to be used on the next xy move, if the head wasn't in the correct z position yet.
     *
     * \see GCodeExport::writeExtrusion(Point, double, double)
     *
     * \note After GCodeExport::writeExtrusion(Point, double, double) has been called currentPosition.z coincides with this value
     */
    coord_t current_layer_z_;
    coord_t is_z_hopped_; //!< The amount by which the print head is currently z hopped, or zero if it is not z hopped. (A z hop is used during travel moves to avoid collision with
                          //!< other layer parts)
    std::optional<ZHopAntiOozing> z_hop_prime_leftover_; //!< The leftover priming from a previous travel move that should be processed while z-hopping down

    size_t current_extruder_;
    std::map<size_t, double> current_fans_speeds_; //!< Current fan speed, by fan index. No value means the speed has never been set yet.
    size_t fans_count_{ 0 };
    EGCodeFlavor flavor_;

    std::vector<Duration> total_print_times_; //!< The total estimated print time in seconds for each feature
    TimeEstimateCalculator estimate_calculator_;

    LayerIndex layer_nr_; //!< for sending travel data

    bool is_volumetric_;
    bool relative_extrusion_; //!< whether to use relative extrusion distances rather than absolute
    bool always_write_active_tool_; //!< whether to write the active tool after sending commands to inactive tool

    Temperature initial_bed_temp_; //!< bed temperature at the beginning of the print.
    Temperature bed_temperature_; //!< Current build plate temperature.
    Temperature build_volume_temperature_; //!< build volume temperature
    bool machine_heated_build_volume_; //!< does the machine have the ability to control/stabilize build-volume-temperature
    bool ppr_enable_; //!< if the print process reporting is enabled

protected:
    /*!
     * Convert an E value to a value in mm (if it wasn't already in mm) for the current extruder.
     *
     * E values are either in mm or in mm^3
     * The current extruder is used to determine the filament area to make the conversion.
     *
     * \param e the value to convert
     * \return the value converted to mm
     */
    double eToMm(double e);

    /*!
     * Convert a distance value to an E value (which might be linear/distance based as well) for the current extruder.
     *
     * E values are either in mm or in mm^3
     * The current extruder is used to determine the filament area to make the conversion.
     *
     * \param mm the value to convert
     * \return the value converted to mm or mm3 depending on whether the E axis is volumetric
     */
    double mmToE(double mm) const;

    /*!
     * Convert an E value to a value in mm3 (if it wasn't already in mm3) for the provided extruder.
     *
     * E values are either in mm or in mm^3
     * The given extruder is used to determine the filament area to make the conversion.
     *
     * \param e the value to convert
     * \param extruder Extruder number
     * \return the value converted to mm3
     */
    double eToMm3(double e, size_t extruder);

public:
    GCodeExport();
    ~GCodeExport();

    /*
     * \brief Converts the g-code flavor to a string as it must be printed in
     * the g-code.
     * \param flavor The g-code flavor to print.
     * \return A serialized form of this flavor.
     */
    static std::string flavorToString(const EGCodeFlavor& flavor);

    /*!
     * Get the gcode file header (e.g. ";FLAVOR:UltiGCode\n")
     *
     * \param extruder_is_used For each extruder whether it is used in the print
     * \param print_time The total print time in seconds of the whole gcode (if known)
     * \param filament_used The total mm^3 filament used for each extruder or a vector of the wrong size of unknown
     * \param mat_ids The material GUIDs for each material.
     * \return The string representing the file header
     */
    std::string getFileHeader(
        const std::vector<bool>& extruder_is_used,
        const Duration* print_time = nullptr,
        const std::vector<double>& filament_used = std::vector<double>(),
        const std::vector<std::string>& mat_ids = std::vector<std::string>());

    void setSliceUUID(const std::string& slice_uuid);

    void setLayerNr(const LayerIndex& layer_nr);

    void setOutputStream(std::ostream* stream);

    bool getExtruderIsUsed(const int extruder_nr) const; //!< return whether the extruder has been used throughout printing all meshgroup up till now

    Point2LL getGcodePos(const coord_t x, const coord_t y, const int extruder_train) const;

    Point2LL getNozzleOffset(const int extruder_train) const;

    void setFlavor(EGCodeFlavor flavor);
    EGCodeFlavor getFlavor() const;

    void setZ(int z);

    void setFlowRateExtrusionSettings(double max_extrusion_offset, double extrusion_offset_factor);

    /*!
     * Add extra amount of material to be primed after an unretraction.
     *
     * \param extra_prime_distance Amount of material in mm.
     */
    void addExtraPrimeAmount(double extra_prime_volume);

    const Point3LL& getPosition() const;

    Point2LL getPositionXY() const;

    int getPositionZ() const;

    int getExtruderNr() const;

    void setFilamentDiameter(size_t extruder, const coord_t diameter);

    double getCurrentExtrudedVolume() const;

    /*!
     * Get the total extruded volume for a specific extruder in mm^3
     *
     * Retractions and unretractions don't contribute to this.
     *
     * \param extruder_nr The extruder number for which to get the total netto extruded volume
     * \return total filament printed in mm^3
     */
    double getTotalFilamentUsed(size_t extruder_nr);

    /*!
     * Get the total estimated print time in seconds for each feature
     *
     * \return total print time in seconds for each feature
     */
    std::vector<Duration> getTotalPrintTimePerFeature();
    /*!
     * Get the total print time in seconds for the complete print
     *
     * \return total print time in seconds for the complete print
     */
    double getSumTotalPrintTimes();
    void updateTotalPrintTime();
    void resetTotalPrintTimeAndFilament();

    void writeComment(const std::string& comment);
    void writeTypeComment(const PrintFeatureType& type);

    /*!
     * Write an M82 (absolute) or M83 (relative)
     *
     * \param set_relative_extrusion_mode If true, write an M83, otherwise write an M82
     */
    void writeExtrusionMode(bool set_relative_extrusion_mode);

    void resetExtrusionMode();

    /*!
     * Write a comment saying what (estimated) time has passed up to this point
     *
     * \param time The time passed up till this point
     */
    void writeTimeComment(const Duration time);

    /*!
     * Write a comment saying that we're starting a certain layer.
     */
    void writeLayerComment(const LayerIndex layer_nr);

    /*!
     * Write a comment saying that the print has a certain number of layers.
     */
    void writeLayerCountComment(const size_t layer_count);

    void writeLine(const char* line);

    /*!
     * Reset the current_e_value to prevent too high E values.
     *
     * The current extruded volume is added to the current extruder_attr.
     */
    void resetExtrusionValue();

    void writeDelay(const Duration& time_amount);

    /*!
     * Coordinates are build plate coordinates, which might be offsetted when extruder offsets are encoded in the gcode.
     *
     * \param p location to go to
     * \param speed movement speed
     */
    void writeTravel(const Point2LL& p, const Velocity& speed);

    /*!
     * Coordinates are build plate coordinates, which might be offsetted when extruder offsets are encoded in the gcode.
     *
     * \param p location to go to
     * \param speed movement speed
     * \param feature the feature that's currently printing
     * \param update_extrusion_offset whether to update the extrusion offset to match the current flow rate
     */
    void writeExtrusion(const Point2LL& p, const Velocity& speed, double extrusion_mm3_per_mm, PrintFeatureType feature, bool update_extrusion_offset = false);

    /*!
     * Go to a X/Y location with the z-hopped Z value
     * Coordinates are build plate coordinates, which might be offsetted when extruder offsets are encoded in the gcode.
     *
     * \param p location to go to
     * \param speed movement speed
     *  \param retract_distance The absolute retraction distance to be reached during this travel, or nullopt to leave it unchanged
     */
    void writeTravel(const Point3LL& p, const Velocity& speed, const std::optional<double> retract_distance = std::nullopt);

    /*!
     * Go to a X/Y location with the extrusion Z
     * Perform un-z-hop
     * Perform unretraction
     *
     * Coordinates are build plate coordinates, which might be offsetted when extruder offsets are encoded in the gcode.
     *
     * \param p location to go to
     * \param speed movement speed
     * \param feature the feature that's currently printing
     * \param update_extrusion_offset whether to update the extrusion offset to match the current flow rate
     */
    void writeExtrusion(const Point3LL& p, const Velocity& speed, double extrusion_mm3_per_mm, PrintFeatureType feature, bool update_extrusion_offset = false);

    /*!
     * Initialize the extruder trains.
     *
     * \param[in] storage where the slice data is stored.
     * \param[in] start_extruder_nr The extruder with which to start the print.
     */
    bool initializeExtruderTrains(const SliceDataStorage& storage, const size_t start_extruder_nr);

    /*!
     * Set temperatures for the initial layer. Called by 'processStartingCode' and whenever a new object is started at layer 0.
     *
     * \param[in] storage where the slice data is stored.
     * \param[in] start_extruder_nr The extruder with which to start the print.
     */
    void processInitialLayerTemperature(const SliceDataStorage& storage, const size_t start_extruder_nr);

    /*!
     * Certain gcode flavors require a prime blob to be printed before the first layer.
     *
     * \param storage where the settings are stored
     * \return whether a prime blob is required
     */
    bool needPrimeBlob() const;

    /*
     *  Function is used to write the content of output_stream to the gcode file
     */
    void flushOutputStream();

    /*!
     * Convert a volume value to an E value (which might be volumetric as well) for the current extruder.
     *
     * E values are either in mm or in mm^3
     * The current extruder is used to determine the filament area to make the conversion.
     *
     * \param mm3 the value to convert
     * \return the value converted to mm or mm3 depending on whether the E axis is volumetric
     */
    double mm3ToE(double mm3) const;

private:
    /*!
     * Coordinates are build plate coordinates, which might be offsetted when extruder offsets are encoded in the gcode.
     *
     * \param x build plate x
     * \param y build plate y
     * \param z build plate z
     * \param speed movement speed
     * \param retract_distance The absolute retraction distance to be reached during this travel, or nullopt to leave it unchanged
     */
    void writeTravel(const coord_t x, const coord_t y, const coord_t z, const Velocity& speed, const std::optional<double> retract_distance = std::nullopt);

    /*!
     * Perform un-z-hop
     * Perform unretract
     * Write extrusion move
     * Coordinates are build plate coordinates, which might be offsetted when extruder offsets are encoded in the gcode.
     *
     * \param x build plate x
     * \param y build plate y
     * \param z build plate z
     * \param speed movement speed
     * \param extrusion_mm3_per_mm flow
     * \param feature the print feature that's currently printing
     * \param update_extrusion_offset whether to update the extrusion offset to match the current flow rate
     */
    void writeExtrusion(
        const coord_t x,
        const coord_t y,
        const coord_t z,
        const Velocity& speed,
        const double extrusion_mm3_per_mm,
        const PrintFeatureType& feature,
        const bool update_extrusion_offset = false);

    /*!
     * Write the F, X, Y, Z and E value (if they are not different from the last)
     *
     * convenience function called from writeExtrusion and writeTravel
     *
     * This function also applies the gcode offset by calling \ref GCodeExport::getGcodePos
     * This function updates the \ref GCodeExport::total_bounding_box
     * It estimates the time in \ref GCodeExport::estimateCalculator for the correct feature
     * It updates \ref GCodeExport::currentPosition, \ref GCodeExport::current_e_value and \ref GCodeExport::currentSpeed
     */
    void writeFXYZE(
        const Velocity& speed,
        const coord_t x,
        const coord_t y,
        const coord_t z,
        const double e,
        const PrintFeatureType& feature,
        const std::optional<RetractionAmounts>& retraction_amounts = std::nullopt);

    /*!
     * The writeTravel and/or writeExtrusion when flavor == BFB
     * \param x build plate x
     * \param y build plate y
     * \param z build plate z
     * \param speed movement speed
     * \param extrusion_mm3_per_mm flow
     * \param feature print feature to track print time for
     */
    void writeMoveBFB(const int x, const int y, const int z, const Velocity& speed, double extrusion_mm3_per_mm, PrintFeatureType feature);

    /*!
     * Set bed temperature for the initial layer. Called by 'processInitialLayerTemperatures'.
     */
    void processInitialLayerBedTemperature();

    /*!
     * Set extruders temperatures for the initial layer. Called by 'processInitialLayerTemperatures'.
     *
     * \param storage The slice data storage
     * \param wait_start_extruder Indicates whether we should always wait for the start extruder temperature to be reached
     * \param start_extruder_nr The index of the start extruder
     */
    void processInitialLayerExtrudersTemperatures(const SliceDataStorage& storage, const bool wait_start_extruder, const size_t start_extruder_nr);

    /*!
     * Write a stationary or travelling retraction/unretraction and set the proper associated internal variables
     * @param retraction_amounts The retraction amounts to be applied
     */
    void writeRawRetract(const RetractionAmounts& retraction_amounts);

    /*!
     * Compute the appropriate retraction amounts according to the given extruder and retraction distance to reach
     * @param extruder_attributes The extruder to be used
     * @param distance The absolute retraction distance to be reached
     * @return The calculated retraction amounts
     */
    RetractionAmounts computeRetractionAmounts(const ExtruderTrainAttributes& extruder_attributes, const double distance) const;

    /*!
     * Start or end a z hop
     * \param speed The speed used for moving, or 0 to take the default speed
     * \param height The actual height to be reached, relative to the current layer height
     * \param retract_distance The absolute retraction distance to be reached while doing the z-hop move, or nullopt to leave it unchanged
     */
    void writeZhop(Velocity speed = 0.0, const coord_t height = 0, const std::optional<double> retract_distance = std::nullopt);

    static PrintFeatureType
        sendTravel(const Point3LL& p, const Velocity& speed, const ExtruderTrainAttributes& extruder_attr, const std::optional<RetractionAmounts>& retraction_amounts);

public:
    /*!
     * Get ready for extrusion moves:
     * - unretract (G11 or G1 E.)
     * - prime blob (G1 E)
     *
     * It estimates the time in \ref GCodeExport::estimateCalculator
     * It updates \ref GCodeExport::current_e_value and \ref GCodeExport::currentSpeed
     */
    void writeUnretractionAndPrime();

    /*!
     * Write a stationary retraction
     * @param config The retraction configuration to be used
     * @param force Indicates whether we should force the retraction to happen regardless of the maximum allowed retraction count
     * @param extruder_switch Indicates whether we retract for an extruder switch
     * @param retract_distance A specific absolute retraction distance to be used, or nullopt to use the one in the config
     * @return True if the retraction has been processed normally, false if it was skipped because of limitations
     */
    bool writeRetraction(const RetractionConfig& config, bool force = false, bool extruder_switch = false, const std::optional<double> retract_distance = std::nullopt);

    /*!
     * Start a z hop with the given \p hop_height.
     *
     * \param hop_height The height to move above the current layer.
     * \param speed The speed used for moving.
     * \param retract_distance The absolute retract distance to be reached during the z-hop move, or nullopt to leave it unchanged
     * \param retract_ratio This is the ratio of the complete z-hop move that should be used to process the retraction. If >0 and <1 then the z-hop move
     *                      will actually be split in two part, one with retraction and one without.
     */
    void writeZhopStart(const coord_t hop_height, Velocity speed = 0.0, std::optional<double> retract_distance = std::nullopt, const Ratio& retract_ratio = 0.0_r);

    /*!
     * End a z hop: go back to the layer height
     *
     * \param speed The speed used for moving.
     * \param prime_distance The absolute prime distance to be reached during the z-hop move, or nullopt to leave it unchanged
     * \param prime_ratio This is the ratio of the complete z-hop move that should be used to process the priming. If >0 and <1 then the z-hop move
     *                    will actually be split in two part, one without priming and one with.
     */
    void writeZhopEnd(Velocity speed = 0.0, const coord_t height = 0, const std::optional<double> prime_distance = std::nullopt, const Ratio& prime_ratio = 0.0_r);

    /*!
     * Start the new_extruder:
     * - set new extruder
     * - zero E value
     * - write extruder start gcode
     *
     * \param new_extruder The extruder to start with
     */
    void startExtruder(const size_t new_extruder);

    /*!
     * Switch to the new_extruder:
     * - perform neccessary retractions
     * - fiddle with E-values
     * - write extruder end gcode
     * - set new extruder
     * - write extruder start gcode
     *
     * \param new_extruder The extruder to switch to
     * \param retraction_config_old_extruder The extruder switch retraction config of the old extruder, to perform the extruder switch retraction with.
     * \param perform_z_hop The amount by which the print head should be z hopped during extruder switch, or zero if it should not z hop.
     */
    void switchExtruder(size_t new_extruder, const RetractionConfig& retraction_config_old_extruder, coord_t perform_z_hop = 0);

    void writeCode(const char* str);

    void resetExtruderToPrimed(const size_t extruder, const double initial_retraction);

    /*!
     * Write the gcode for priming the current extruder train so that it can be used.
     *
     * \param travel_speed The travel speed when priming involves a movement
     */
    void writePrimeTrain(const Velocity& travel_speed);

    /*!
     * \brief Write a set fan speed command, if different from the actual speed
     * \param speed The new fan speed, which should be [0.0, 100.0]
     * \param extruder The extruder for which we want to set the cooling fan speed, or nullopt to use the current extruder
     */
    void writeFanCommand(double speed, std::optional<size_t> extruder = std::nullopt);

    /*!
     * \brief Write a set fan speed command for the given fan, if different from the actual speed
     * \param speed The new fan speed, which should be [0.0, 100.0]
     * \param fan_number The fan for which we want to set the speed
     */
    void writeSpecificFanCommand(double speed, size_t fan_number);

    /*! Write cooling fan speeds before proceeding an extruder switch */
    void writePrepareFansForNozzleSwitch();

    /*!
     * \brief Write the cooling fan speeds before starting an actual extrusion
     * \param current_extruder_new_speed The new speed for the currently active extruder
     * \note All other cooling fans but the active one will be deactivaed
     */
    void writePrepareFansForExtrusion(double current_extruder_new_speed);

    /*!
     * \brief Write a GCode temperature command
     * \param extruder The extruder number
     * \param temperature The temperature to bo set
     * \param wait Indicates whether we should just set the temperature and keep going, or wait for the temperature to be reach before going further
     * \param force_write_on_equal When true, we should write the temperature command even if the actual set temperature is the same
     */
    void writeTemperatureCommand(const size_t extruder, const Temperature& temperature, const bool wait = false, const bool force_write_on_equal = false);
    void writeBedTemperatureCommand(const Temperature& temperature, const bool wait = false);
    void writeBuildVolumeTemperatureCommand(const Temperature& temperature, const bool wait = false);

    /*!
     * Write the command for setting the acceleration for print moves to a specific value
     */
    void writePrintAcceleration(const Acceleration& acceleration);

    /*!
     * Write the command for setting the acceleration for travel moves to a specific value
     */
    void writeTravelAcceleration(const Acceleration& acceleration);

    /*!
     * Write the command for setting the jerk to a specific value
     */
    void writeJerk(const Velocity& jerk);

    /*!
     * Set member variables using the settings in \p settings.
     */
    void preSetup(const size_t start_extruder);

    /*!
     * Handle the initial (bed/nozzle) temperatures before any gcode is processed.
     * These temperatures are set in the pre-print setup in the firmware.
     *
     * See FffGcodeWriter::processStartingCode
     * \param start_extruder_nr The extruder with which to start this print
     */
    void setInitialAndBuildVolumeTemps(const unsigned int start_extruder_nr);

    /*!
     * Override or set an initial nozzle temperature as written by GCodeExport::setInitialTemps
     * This is used primarily during better specification of temperatures in LayerPlanBuffer::insertPreheatCommand
     *
     * \warning This function must be called before any of the layers in the meshgroup are written to file!
     * That's because it sets the current temperature in the gcode!
     *
     * \param extruder_nr The extruder number for which to better specify the temp
     * \param temp The temp at which the nozzle should be at startup
     */
    void setInitialTemp(int extruder_nr, double temp);

    /*!
     * Finish the gcode: turn fans off, write end gcode and flush all gcode left in the buffer.
     *
     * \param endCode The end gcode to be appended at the very end.
     */
    void finalize(const char* endCode);

    /*!
     * Get amount of material extruded since last wipe script was inserted.
     *
     * \param extruder Extruder number to check.
     */
    double getExtrudedVolumeAfterLastWipe(size_t extruder);

    /*!
     *  Reset the last_e_value_after_wipe.
     *
     * \param extruder Extruder number which last_e_value_after_wipe value to reset.
     */
    void ResetLastEValueAfterWipe(size_t extruder);

    /*!
     *  Generate g-code for wiping current nozzle using provided config.
     *
     * \param wipe_config Config with wipe script settings.
     */
    void insertWipeScript(const WipeScriptConfig& wipe_config);

    /*!
     * Set the priming leftover to be processed during the next z-hop end
     */
    void setZHopPrimeLeftover(const ZHopAntiOozing& z_hop_prime_leftover);

    /*!
     * Indicates whether the printer handles the retraction/priming, totally or with specific commands
     */
    bool machineHandlesRetraction() const;
};

} // namespace cura

#endif // GCODEEXPORT_H
