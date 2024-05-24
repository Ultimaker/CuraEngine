// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "gcodeExport.h"

#include <cassert>
#include <cmath>
#include <iomanip>
#include <numbers>

#include <spdlog/spdlog.h>

#include "Application.h" //To send layer view data.
#include "ExtruderTrain.h"
#include "PrintFeature.h"
#include "RetractionConfig.h"
#include "Slice.h"
#include "WipeScriptConfig.h"
#include "communication/Communication.h" //To send layer view data.
#include "geometry/OpenPolyline.h"
#include "geometry/Point2D.h"
#include "settings/types/LayerIndex.h"
#include "utils/Date.h"
#include "utils/linearAlg2D.h"
#include "utils/string.h" // MMtoStream, PrecisionedDouble

namespace cura
{

std::string transliterate(const std::string& text)
{
    // For now, just replace all non-ascii characters with '?'.
    // This function can be expanded if we need more complex transliteration.
    std::ostringstream stream;
    for (const char& c : text)
    {
        stream << ((c >= 0) ? c : '?');
    }
    return stream.str();
}

GCodeExport::GCodeExport()
    : output_stream_(&std::cout)
    , current_position_(0, 0, MM2INT(20))
    , layer_nr_(0)
    , relative_extrusion_(false)
{
    *output_stream_ << std::fixed;

    current_e_value_ = 0;
    current_extruder_ = 0;
    current_fan_speed_ = -1;

    total_print_times_ = std::vector<Duration>(static_cast<unsigned char>(PrintFeatureType::NumPrintFeatureTypes), 0.0);

    current_speed_ = 1.0;
    current_print_acceleration_ = -1.0;
    current_travel_acceleration_ = -1.0;
    current_jerk_ = -1.0;

    is_z_hopped_ = 0;
    setFlavor(EGCodeFlavor::MARLIN);
    initial_bed_temp_ = 0;
    bed_temperature_ = 0;
    build_volume_temperature_ = 0;
    machine_heated_build_volume_ = false;
    ppr_enable_ = false;

    fan_number_ = 0;
    use_extruder_offset_to_offset_coords_ = false;
    machine_name_ = "";
    relative_extrusion_ = false;
    new_line_ = "\n";

    total_bounding_box_ = AABB3D();
}

GCodeExport::~GCodeExport()
{
}

void GCodeExport::preSetup(const size_t start_extruder)
{
    current_extruder_ = start_extruder;

    const Scene& scene = Application::getInstance().current_slice_->scene;
    std::vector<MeshGroup>::iterator mesh_group = scene.current_mesh_group;
    setFlavor(mesh_group->settings.get<EGCodeFlavor>("machine_gcode_flavor"));
    use_extruder_offset_to_offset_coords_ = mesh_group->settings.get<bool>("machine_use_extruder_offset_to_offset_coords");
    const size_t extruder_count = Application::getInstance().current_slice_->scene.extruders.size();
    ppr_enable_ = mesh_group->settings.get<bool>("ppr_enable");

    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        const ExtruderTrain& train = scene.extruders[extruder_nr];
        setFilamentDiameter(extruder_nr, train.settings_.get<coord_t>("material_diameter"));

        extruder_attr_[extruder_nr].last_retraction_prime_speed_
            = train.settings_.get<Velocity>("retraction_prime_speed"); // the alternative would be switch_extruder_prime_speed, but dual extrusion might not even be configured...
        extruder_attr_[extruder_nr].fan_number_ = train.settings_.get<size_t>("machine_extruder_cooling_fan_number");

        // Cache some settings that we use frequently.
        const Settings& extruder_settings = Application::getInstance().current_slice_->scene.extruders[extruder_nr].settings_;
        if (use_extruder_offset_to_offset_coords_)
        {
            extruder_attr_[extruder_nr].nozzle_offset_
                = Point2LL(extruder_settings.get<coord_t>("machine_nozzle_offset_x"), extruder_settings.get<coord_t>("machine_nozzle_offset_y"));
        }
        else
        {
            extruder_attr_[extruder_nr].nozzle_offset_ = Point2LL(0, 0);
        }
        extruder_attr_[extruder_nr].machine_firmware_retract_ = extruder_settings.get<bool>("machine_firmware_retract");
    }

    machine_name_ = mesh_group->settings.get<std::string>("machine_name");

    relative_extrusion_ = mesh_group->settings.get<bool>("relative_extrusion");
    always_write_active_tool_ = mesh_group->settings.get<bool>("machine_always_write_active_tool");

    if (flavor_ == EGCodeFlavor::BFB)
    {
        new_line_ = "\r\n";
    }
    else
    {
        new_line_ = "\n";
    }

    estimate_calculator_.setFirmwareDefaults(mesh_group->settings);

    if (mesh_group == scene.mesh_groups.begin())
    {
        if (! scene.current_mesh_group->settings.get<bool>("material_bed_temp_prepend"))
        {
            // Current bed temperature is the one of the first layer (has already been set in header)
            bed_temperature_ = scene.current_mesh_group->settings.get<Temperature>("material_bed_temperature_layer_0");
        }
        else
        {
            // Bed temperature has not been set yet
        }
    }
    else
    {
        // Current bed temperature is the one of the previous group
        bed_temperature_ = (scene.current_mesh_group - 1)->settings.get<Temperature>("material_bed_temperature");
    }
}

void GCodeExport::setInitialAndBuildVolumeTemps(const unsigned int start_extruder_nr)
{
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const size_t extruder_count = Application::getInstance().current_slice_->scene.extruders.size();
    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        const ExtruderTrain& train = scene.extruders[extruder_nr];

        const Temperature print_temp_0 = train.settings_.get<Temperature>("material_print_temperature_layer_0");
        const Temperature print_temp_here = (print_temp_0 != 0) ? print_temp_0 : train.settings_.get<Temperature>("material_print_temperature");
        const Temperature temp = (extruder_nr == start_extruder_nr) ? print_temp_here : train.settings_.get<Temperature>("material_standby_temperature");
        setInitialTemp(extruder_nr, temp);
    }

    initial_bed_temp_ = scene.current_mesh_group->settings.get<Temperature>("material_bed_temperature_layer_0");
    machine_heated_build_volume_ = scene.current_mesh_group->settings.get<bool>("machine_heated_build_volume");
    build_volume_temperature_ = machine_heated_build_volume_ ? scene.current_mesh_group->settings.get<Temperature>("build_volume_temperature") : Temperature(0);
}

void GCodeExport::setInitialTemp(int extruder_nr, double temp)
{
    extruder_attr_[extruder_nr].initial_temp_ = temp;
    if (flavor_ == EGCodeFlavor::GRIFFIN || flavor_ == EGCodeFlavor::ULTIGCODE)
    {
        extruder_attr_[extruder_nr].current_temperature_ = temp;
    }
}

std::string GCodeExport::flavorToString(const EGCodeFlavor& flavor)
{
    switch (flavor)
    {
    case EGCodeFlavor::BFB:
        return "BFB";
    case EGCodeFlavor::MACH3:
        return "Mach3";
    case EGCodeFlavor::MAKERBOT:
        return "Makerbot";
    case EGCodeFlavor::ULTIGCODE:
        return "UltiGCode";
    case EGCodeFlavor::MARLIN_VOLUMATRIC:
        return "Marlin(Volumetric)";
    case EGCodeFlavor::GRIFFIN:
        return "Griffin";
    case EGCodeFlavor::REPETIER:
        return "Repetier";
    case EGCodeFlavor::REPRAP:
        return "RepRap";
    case EGCodeFlavor::MARLIN:
    default:
        return "Marlin";
    }
}

std::string GCodeExport::getFileHeader(
    const std::vector<bool>& extruder_is_used,
    const Duration* print_time,
    const std::vector<double>& filament_used,
    const std::vector<std::string>& mat_ids)
{
    std::ostringstream prefix;

    const size_t extruder_count = Application::getInstance().current_slice_->scene.extruders.size();
    switch (flavor_)
    {
    case EGCodeFlavor::GRIFFIN:
        prefix << ";START_OF_HEADER" << new_line_;
        prefix << ";HEADER_VERSION:0.1" << new_line_;
        prefix << ";FLAVOR:" << flavorToString(flavor_) << new_line_;
        prefix << ";GENERATOR.NAME:Cura_SteamEngine" << new_line_;
        prefix << ";GENERATOR.VERSION:" << CURA_ENGINE_VERSION << new_line_;
        prefix << ";GENERATOR.BUILD_DATE:" << Date::getDate().toStringDashed() << new_line_;
        prefix << ";TARGET_MACHINE.NAME:" << transliterate(machine_name_) << new_line_;

        for (size_t extr_nr = 0; extr_nr < extruder_count; extr_nr++)
        {
            if (! extruder_is_used[extr_nr])
            {
                continue;
            }
            prefix << ";EXTRUDER_TRAIN." << extr_nr << ".INITIAL_TEMPERATURE:" << extruder_attr_[extr_nr].initial_temp_ << new_line_;
            if (filament_used.size() == extruder_count)
            {
                prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.VOLUME_USED:" << static_cast<int>(filament_used[extr_nr]) << new_line_;
            }
            if (mat_ids.size() == extruder_count && mat_ids[extr_nr] != "")
            {
                prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.GUID:" << mat_ids[extr_nr] << new_line_;
            }
            const Settings& extruder_settings = Application::getInstance().current_slice_->scene.extruders[extr_nr].settings_;
            prefix << ";EXTRUDER_TRAIN." << extr_nr << ".NOZZLE.DIAMETER:" << extruder_settings.get<double>("machine_nozzle_size") << new_line_;
            prefix << ";EXTRUDER_TRAIN." << extr_nr << ".NOZZLE.NAME:" << extruder_settings.get<std::string>("machine_nozzle_id") << new_line_;
        }
        prefix << ";BUILD_PLATE.INITIAL_TEMPERATURE:" << initial_bed_temp_ << new_line_;

        if (machine_heated_build_volume_)
        {
            prefix << ";BUILD_VOLUME.TEMPERATURE:" << build_volume_temperature_ << new_line_;
        }

        if (print_time)
        {
            prefix << ";PRINT.TIME:" << static_cast<int>(*print_time) << new_line_;
        }

        prefix << ";PRINT.GROUPS:" << Application::getInstance().current_slice_->scene.mesh_groups.size() << new_line_;

        if (total_bounding_box_.min_.x_ > total_bounding_box_.max_.x_) // We haven't encountered any movement (yet). This probably means we're command-line slicing.
        {
            // Put some small default in there.
            total_bounding_box_.min_ = Point3LL(0, 0, 0);
            total_bounding_box_.max_ = Point3LL(10, 10, 10);
        }
        prefix << ";PRINT.SIZE.MIN.X:" << INT2MM(total_bounding_box_.min_.x_) << new_line_;
        prefix << ";PRINT.SIZE.MIN.Y:" << INT2MM(total_bounding_box_.min_.y_) << new_line_;
        prefix << ";PRINT.SIZE.MIN.Z:" << INT2MM(total_bounding_box_.min_.z_) << new_line_;
        prefix << ";PRINT.SIZE.MAX.X:" << INT2MM(total_bounding_box_.max_.x_) << new_line_;
        prefix << ";PRINT.SIZE.MAX.Y:" << INT2MM(total_bounding_box_.max_.y_) << new_line_;
        prefix << ";PRINT.SIZE.MAX.Z:" << INT2MM(total_bounding_box_.max_.z_) << new_line_;
        prefix << ";SLICE_UUID:" << slice_uuid_ << new_line_;

        if (ppr_enable_)
        {
            prefix << ";PPR_ENABLE:TRUE" << new_line_;

            for (size_t extr_nr = 0; extr_nr < extruder_count; extr_nr++)
            {
                if (! extruder_is_used[extr_nr])
                {
                    continue;
                }
                const Settings& extruder_settings = Application::getInstance().current_slice_->scene.extruders[extr_nr].settings_;
                prefix << ";EXTRUDER_TRAIN." << extr_nr << ".PPR_FLOW_WARNING:" << extruder_settings.get<double>("flow_warn_limit") << new_line_;
                prefix << ";EXTRUDER_TRAIN." << extr_nr << ".PPR_FLOW_LIMIT:" << extruder_settings.get<double>("flow_anomaly_limit") << new_line_;
                prefix << ";EXTRUDER_TRAIN." << extr_nr << ".PPR_PRINTING_TEMPERATURE_WARNING:" << extruder_settings.get<double>("print_temp_warn_limit") << new_line_;
                prefix << ";EXTRUDER_TRAIN." << extr_nr << ".PPR_PRINTING_TEMPERATURE_LIMIT:" << extruder_settings.get<double>("print_temp_anomaly_limit") << new_line_;
            }
            prefix << ";PPR_BUILD_VOLUME_TEMPERATURE_WARNING:" << Application::getInstance().current_slice_->scene.extruders[0].settings_.get<double>("bv_temp_warn_limit")
                   << new_line_;
            prefix << ";PPR_BUILD_VOLUME_TEMPERATURE_LIMIT:" << Application::getInstance().current_slice_->scene.extruders[0].settings_.get<double>("bv_temp_anomaly_limit")
                   << new_line_;
        }
        prefix << ";END_OF_HEADER" << new_line_;
        break;
    default:
        prefix << ";FLAVOR:" << flavorToString(flavor_) << new_line_;
        prefix << ";TIME:" << ((print_time) ? static_cast<int>(*print_time) : 6666) << new_line_;
        if (flavor_ == EGCodeFlavor::ULTIGCODE)
        {
            prefix << ";MATERIAL:" << ((filament_used.size() >= 1) ? static_cast<int>(filament_used[0]) : 6666) << new_line_;
            prefix << ";MATERIAL2:" << ((filament_used.size() >= 2) ? static_cast<int>(filament_used[1]) : 0) << new_line_;

            prefix << ";NOZZLE_DIAMETER:" << Application::getInstance().current_slice_->scene.extruders[0].settings_.get<double>("machine_nozzle_size") << new_line_;
        }
        else if (flavor_ == EGCodeFlavor::REPRAP || flavor_ == EGCodeFlavor::MARLIN || flavor_ == EGCodeFlavor::MARLIN_VOLUMATRIC)
        {
            prefix << ";Filament used: ";
            if (filament_used.size() > 0)
            {
                for (unsigned i = 0; i < filament_used.size(); ++i)
                {
                    if (i > 0)
                    {
                        prefix << ", ";
                    }
                    if (flavor_ != EGCodeFlavor::MARLIN_VOLUMATRIC)
                    {
                        prefix << filament_used[i] / (1000 * extruder_attr_[i].filament_area_) << "m";
                    }
                    else // Use volumetric filament used.
                    {
                        prefix << filament_used[i] << "mm3";
                    }
                }
            }
            else
            {
                prefix << "0m";
            }
            prefix << new_line_;
            prefix << ";Layer height: " << Application::getInstance().current_slice_->scene.current_mesh_group->settings.get<double>("layer_height") << new_line_;
        }
        prefix << ";MINX:" << INT2MM(total_bounding_box_.min_.x_) << new_line_;
        prefix << ";MINY:" << INT2MM(total_bounding_box_.min_.y_) << new_line_;
        prefix << ";MINZ:" << INT2MM(total_bounding_box_.min_.z_) << new_line_;
        prefix << ";MAXX:" << INT2MM(total_bounding_box_.max_.x_) << new_line_;
        prefix << ";MAXY:" << INT2MM(total_bounding_box_.max_.y_) << new_line_;
        prefix << ";MAXZ:" << INT2MM(total_bounding_box_.max_.z_) << new_line_;
        prefix << ";TARGET_MACHINE.NAME:" << transliterate(machine_name_) << new_line_;
    }

    return prefix.str();
}


void GCodeExport::setLayerNr(const LayerIndex& layer_nr)
{
    layer_nr_ = layer_nr;
}

void GCodeExport::setOutputStream(std::ostream* stream)
{
    output_stream_ = stream;
    *output_stream_ << std::fixed;
}

bool GCodeExport::getExtruderIsUsed(const int extruder_nr) const
{
    assert(extruder_nr >= 0);
    assert(extruder_nr < MAX_EXTRUDERS);
    return extruder_attr_[extruder_nr].is_used_;
}

Point2LL GCodeExport::getGcodePos(const coord_t x, const coord_t y, const size_t extruder_train) const
{
    return Point2LL(x, y) - extruder_attr_[extruder_train].nozzle_offset_;
}

void GCodeExport::setFlavor(EGCodeFlavor flavor)
{
    this->flavor_ = flavor;
    if (flavor == EGCodeFlavor::MACH3)
    {
        for (int n = 0; n < MAX_EXTRUDERS; n++)
        {
            extruder_attr_[n].extruder_character_ = 'A' + n;
        }
    }
    else
    {
        for (int n = 0; n < MAX_EXTRUDERS; n++)
        {
            extruder_attr_[n].extruder_character_ = 'E';
        }
    }
    if (flavor == EGCodeFlavor::ULTIGCODE || flavor == EGCodeFlavor::MARLIN_VOLUMATRIC)
    {
        is_volumetric_ = true;
    }
    else
    {
        is_volumetric_ = false;
    }
}

EGCodeFlavor GCodeExport::getFlavor() const
{
    return flavor_;
}

void GCodeExport::setZ(int z)
{
    current_layer_z_ = z;
}

void GCodeExport::addExtraPrimeAmount(double extra_prime_volume)
{
    extruder_attr_[current_extruder_].prime_volume_ += extra_prime_volume;
}

void GCodeExport::setFlowRateExtrusionSettings(double max_extrusion_offset, double extrusion_offset_factor)
{
    this->max_extrusion_offset_ = max_extrusion_offset;
    this->extrusion_offset_factor_ = extrusion_offset_factor;
}

const Point3LL& GCodeExport::getPosition() const
{
    return current_position_;
}
Point2LL GCodeExport::getPositionXY() const
{
    return Point2LL(current_position_.x_, current_position_.y_);
}

int GCodeExport::getPositionZ() const
{
    return current_position_.z_;
}

int GCodeExport::getExtruderNr() const
{
    return current_extruder_;
}

void GCodeExport::setFilamentDiameter(const size_t extruder, const coord_t diameter)
{
    const double r = INT2MM(diameter) / 2.0;
    const double area = std::numbers::pi * r * r;
    extruder_attr_[extruder].filament_area_ = area;
}

double GCodeExport::getCurrentExtrudedVolume() const
{
    double extrusion_amount = current_e_value_;
    if (! extruder_attr_[current_extruder_].machine_firmware_retract_)
    { // no E values are changed to perform a retraction
        extrusion_amount
            -= extruder_attr_[current_extruder_].retraction_e_amount_at_e_start_; // subtract the increment in E which was used for the first unretraction instead of extrusion
        extrusion_amount
            += extruder_attr_[current_extruder_].retraction_e_amount_current_; // add the decrement in E which the filament is behind on extrusion due to the last retraction
    }
    if (is_volumetric_)
    {
        return extrusion_amount;
    }
    else
    {
        return extrusion_amount * extruder_attr_[current_extruder_].filament_area_;
    }
}

double GCodeExport::eToMm(double e)
{
    if (is_volumetric_)
    {
        return e / extruder_attr_[current_extruder_].filament_area_;
    }
    else
    {
        return e;
    }
}

double GCodeExport::mm3ToE(double mm3)
{
    if (is_volumetric_)
    {
        return mm3;
    }
    else
    {
        return mm3ToMm(mm3);
    }
}

double GCodeExport::mm3ToMm(double mm3)
{
    return mm3 / extruder_attr_[current_extruder_].filament_area_;
}

double GCodeExport::mmToE(double mm)
{
    if (is_volumetric_)
    {
        return mm * extruder_attr_[current_extruder_].filament_area_;
    }
    else
    {
        return mm;
    }
}

double GCodeExport::eToMm3(double e, size_t extruder)
{
    if (is_volumetric_)
    {
        return e;
    }
    else
    {
        return e * extruder_attr_[extruder].filament_area_;
    }
}

double GCodeExport::getTotalFilamentUsed(size_t extruder_nr)
{
    if (extruder_nr == current_extruder_)
        return extruder_attr_[extruder_nr].total_filament_ + getCurrentExtrudedVolume();
    return extruder_attr_[extruder_nr].total_filament_;
}

std::vector<Duration> GCodeExport::getTotalPrintTimePerFeature()
{
    return total_print_times_;
}

double GCodeExport::getSumTotalPrintTimes()
{
    double sum = 0.0;
    for (double item : getTotalPrintTimePerFeature())
    {
        sum += item;
    }
    return sum;
}

void GCodeExport::resetTotalPrintTimeAndFilament()
{
    for (size_t i = 0; i < total_print_times_.size(); i++)
    {
        total_print_times_[i] = 0.0;
    }
    for (unsigned int e = 0; e < MAX_EXTRUDERS; e++)
    {
        extruder_attr_[e].total_filament_ = 0.0;
        extruder_attr_[e].current_temperature_ = 0;
        extruder_attr_[e].waited_for_temperature_ = false;
    }
    current_e_value_ = 0.0;
    estimate_calculator_.reset();
}

void GCodeExport::updateTotalPrintTime()
{
    std::vector<Duration> estimates = estimate_calculator_.calculate();
    for (size_t i = 0; i < estimates.size(); i++)
    {
        total_print_times_[i] += estimates[i];
    }
    estimate_calculator_.reset();
    writeTimeComment(getSumTotalPrintTimes());
}

void GCodeExport::writeComment(const std::string& unsanitized_comment)
{
    const std::string comment = transliterate(unsanitized_comment);

    *output_stream_ << ";";
    for (unsigned int i = 0; i < comment.length(); i++)
    {
        if (comment[i] == '\n')
        {
            *output_stream_ << new_line_ << ";";
        }
        else
        {
            *output_stream_ << comment[i];
        }
    }
    *output_stream_ << new_line_;
}

void GCodeExport::writeTimeComment(const Duration time)
{
    *output_stream_ << ";TIME_ELAPSED:" << time << new_line_;
}

void GCodeExport::writeTypeComment(const PrintFeatureType& type)
{
    switch (type)
    {
    case PrintFeatureType::OuterWall:
        *output_stream_ << ";TYPE:WALL-OUTER" << new_line_;
        break;
    case PrintFeatureType::InnerWall:
        *output_stream_ << ";TYPE:WALL-INNER" << new_line_;
        break;
    case PrintFeatureType::Skin:
        *output_stream_ << ";TYPE:SKIN" << new_line_;
        break;
    case PrintFeatureType::Support:
        *output_stream_ << ";TYPE:SUPPORT" << new_line_;
        break;
    case PrintFeatureType::SkirtBrim:
        *output_stream_ << ";TYPE:SKIRT" << new_line_;
        break;
    case PrintFeatureType::Infill:
        *output_stream_ << ";TYPE:FILL" << new_line_;
        break;
    case PrintFeatureType::SupportInfill:
        *output_stream_ << ";TYPE:SUPPORT" << new_line_;
        break;
    case PrintFeatureType::SupportInterface:
        *output_stream_ << ";TYPE:SUPPORT-INTERFACE" << new_line_;
        break;
    case PrintFeatureType::PrimeTower:
        *output_stream_ << ";TYPE:PRIME-TOWER" << new_line_;
        break;
    case PrintFeatureType::MoveCombing:
    case PrintFeatureType::MoveRetraction:
    case PrintFeatureType::MoveUnretraction:
    case PrintFeatureType::NoneType:
    case PrintFeatureType::NumPrintFeatureTypes:
        // do nothing
        break;
    }
}


void GCodeExport::writeLayerComment(const LayerIndex layer_nr)
{
    *output_stream_ << ";LAYER:" << layer_nr << new_line_;
}

void GCodeExport::writeLayerCountComment(const size_t layer_count)
{
    *output_stream_ << ";LAYER_COUNT:" << layer_count << new_line_;
}

void GCodeExport::writeLine(const char* line)
{
    *output_stream_ << line << new_line_;
}

void GCodeExport::resetExtrusionMode()
{
    const bool set_relative_extrusion_mode = getFlavor() == EGCodeFlavor::REPRAP;
    writeExtrusionMode(set_relative_extrusion_mode);
}

void GCodeExport::writeExtrusionMode(bool set_relative_extrusion_mode)
{
    if (set_relative_extrusion_mode)
    {
        *output_stream_ << "M83 ;relative extrusion mode" << new_line_;
    }
    else
    {
        *output_stream_ << "M82 ;absolute extrusion mode" << new_line_;
    }
}

void GCodeExport::resetExtrusionValue()
{
    if (! relative_extrusion_)
    {
        *output_stream_ << "G92 " << extruder_attr_[current_extruder_].extruder_character_ << "0" << new_line_;
    }
    double current_extruded_volume = getCurrentExtrudedVolume();
    extruder_attr_[current_extruder_].total_filament_ += current_extruded_volume;
    for (double& extruded_volume_at_retraction : extruder_attr_[current_extruder_].extruded_volume_at_previous_n_retractions_)
    { // update the extruded_volume_at_previous_n_retractions only of the current extruder, since other extruders don't extrude the current volume
        extruded_volume_at_retraction -= current_extruded_volume;
    }
    current_e_value_ = 0.0;
    extruder_attr_[current_extruder_].retraction_e_amount_at_e_start_ = extruder_attr_[current_extruder_].retraction_e_amount_current_;
}

bool GCodeExport::initializeExtruderTrains(const SliceDataStorage& storage, const size_t start_extruder_nr)
{
    bool should_prime_extruder = true;
    const Settings& mesh_group_settings = Application::getInstance().current_slice_->scene.current_mesh_group->settings;

    if (Application::getInstance().communication_->isSequential()) // If we must output the g-code sequentially, we must already place the g-code header here even if we don't know
                                                                   // the exact time/material usages yet.
    {
        std::string prefix = getFileHeader(storage.getExtrudersUsed());
        writeCode(prefix.c_str());
    }

    writeComment("Generated with Cura_SteamEngine " CURA_ENGINE_VERSION);

    if (getFlavor() == EGCodeFlavor::GRIFFIN)
    {
        std::ostringstream tmp;
        tmp << "T" << start_extruder_nr;
        writeLine(tmp.str().c_str());
    }
    else
    {
        processInitialLayerTemperature(storage, start_extruder_nr);
    }

    writeExtrusionMode(false); // ensure absolute extrusion mode is set before the start gcode
    writeCode(mesh_group_settings.get<std::string>("machine_start_gcode").c_str());

    // in case of shared nozzle assume that the machine-start gcode reset the extruders as per machine description
    if (Application::getInstance().current_slice_->scene.settings.get<bool>("machine_extruders_share_nozzle"))
    {
        for (const ExtruderTrain& train : Application::getInstance().current_slice_->scene.extruders)
        {
            resetExtruderToPrimed(train.extruder_nr_, train.settings_.get<double>("machine_extruders_shared_nozzle_initial_retraction"));
        }
    }

    if (mesh_group_settings.get<bool>("machine_heated_build_volume"))
    {
        writeBuildVolumeTemperatureCommand(mesh_group_settings.get<Temperature>("build_volume_temperature"));
    }

    Application::getInstance().communication_->sendCurrentPosition(getPositionXY());
    startExtruder(start_extruder_nr);

    if (getFlavor() == EGCodeFlavor::BFB)
    {
        writeComment("enable auto-retraction");
        std::ostringstream tmp;
        tmp << "M227 S" << (mesh_group_settings.get<coord_t>("retraction_amount") * 2560 / 1000) << " P" << (mesh_group_settings.get<coord_t>("retraction_amount") * 2560 / 1000);
        writeLine(tmp.str().c_str());
    }
    else if (getFlavor() == EGCodeFlavor::GRIFFIN)
    { // initialize extruder trains
        ExtruderTrain& train = Application::getInstance().current_slice_->scene.extruders[start_extruder_nr];
        processInitialLayerTemperature(storage, start_extruder_nr);
        writePrimeTrain(train.settings_.get<Velocity>("speed_travel"));
        should_prime_extruder = false;
        const RetractionConfig& retraction_config = storage.retraction_wipe_config_per_extruder[start_extruder_nr].retraction_config;
        writeRetraction(retraction_config);
    }

    if (mesh_group_settings.get<bool>("relative_extrusion"))
    {
        writeExtrusionMode(true);
    }
    if (getFlavor() != EGCodeFlavor::GRIFFIN)
    {
        if (mesh_group_settings.get<bool>("retraction_enable"))
        {
            // ensure extruder is zeroed
            resetExtrusionValue();

            // retract before first travel move
            writeRetraction(storage.retraction_wipe_config_per_extruder[start_extruder_nr].retraction_config);
        }
    }

    setExtruderFanNumber(start_extruder_nr);

    return should_prime_extruder;
}

void GCodeExport::processInitialLayerBedTemperature()
{
    const Scene& scene = Application::getInstance().current_slice_->scene;
    const bool heated = scene.current_mesh_group->settings.get<bool>("machine_heated_bed");
    const Temperature bed_temp = scene.current_mesh_group->settings.get<Temperature>("material_bed_temperature_layer_0");
    if (heated && bed_temp != 0)
    {
        writeBedTemperatureCommand(bed_temp, scene.current_mesh_group->settings.get<bool>("material_bed_temp_wait"));
    }
}

void GCodeExport::sendTravelLine(const Point2LL& pos, PrintFeatureType line_type, const Velocity& speed)
{
    const int display_width = (line_type == PrintFeatureType::MoveRetraction || line_type == PrintFeatureType::MoveUnretraction) ? MM2INT(0.2) : MM2INT(0.1);
    const double layer_height = Application::getInstance().current_slice_->scene.current_mesh_group->settings.get<double>("layer_height");

    Application::getInstance().communication_->sendLineTo(line_type, pos, display_width, layer_height, speed);
}

void GCodeExport::processInitialLayerTemperature(const SliceDataStorage& storage, const size_t start_extruder_nr)
{
    Scene& scene = Application::getInstance().current_slice_->scene;
    const size_t num_extruders = scene.extruders.size();

    if (getFlavor() == EGCodeFlavor::GRIFFIN)
    {
        processInitialLayerBedTemperature();

        ExtruderTrain& train = scene.extruders[start_extruder_nr];
        constexpr bool wait = true;
        const Temperature print_temp_0 = train.settings_.get<Temperature>("material_print_temperature_layer_0");
        const Temperature print_temp_here = (print_temp_0 != 0) ? print_temp_0 : train.settings_.get<Temperature>("material_print_temperature");
        writeTemperatureCommand(start_extruder_nr, print_temp_here, wait);
    }
    else if (getFlavor() != EGCodeFlavor::ULTIGCODE)
    {
        if (num_extruders > 1 || getFlavor() == EGCodeFlavor::REPRAP)
        {
            std::ostringstream tmp;
            tmp << "T" << start_extruder_nr;
            writeLine(tmp.str().c_str());
        }

        processInitialLayerBedTemperature();

        if (scene.current_mesh_group->settings.get<bool>("material_print_temp_prepend") || (scene.current_mesh_group != scene.mesh_groups.begin()))
        {
            for (unsigned extruder_nr = 0; extruder_nr < num_extruders; extruder_nr++)
            {
                if (storage.getExtrudersUsed()[extruder_nr])
                {
                    const ExtruderTrain& train = scene.extruders[extruder_nr];
                    Temperature extruder_temp;
                    if (extruder_nr == start_extruder_nr)
                    {
                        const Temperature print_temp_0 = train.settings_.get<Temperature>("material_print_temperature_layer_0");
                        extruder_temp = (print_temp_0 != 0) ? print_temp_0 : train.settings_.get<Temperature>("material_print_temperature");
                    }
                    else
                    {
                        extruder_temp = train.settings_.get<Temperature>("material_standby_temperature");
                    }
                    writeTemperatureCommand(extruder_nr, extruder_temp);
                }
            }
            if (scene.current_mesh_group->settings.get<bool>("material_print_temp_wait"))
            {
                for (unsigned extruder_nr = 0; extruder_nr < num_extruders; extruder_nr++)
                {
                    if (storage.getExtrudersUsed()[extruder_nr])
                    {
                        const ExtruderTrain& train = scene.extruders[extruder_nr];
                        Temperature extruder_temp;
                        if (extruder_nr == start_extruder_nr)
                        {
                            const Temperature print_temp_0 = train.settings_.get<Temperature>("material_print_temperature_layer_0");
                            extruder_temp = (print_temp_0 != 0) ? print_temp_0 : train.settings_.get<Temperature>("material_print_temperature");
                        }
                        else
                        {
                            extruder_temp = train.settings_.get<Temperature>("material_standby_temperature");
                        }
                        writeTemperatureCommand(extruder_nr, extruder_temp, true);
                    }
                }
            }
        }
    }
}

bool GCodeExport::needPrimeBlob() const
{
    switch (getFlavor())
    {
    case EGCodeFlavor::GRIFFIN:
        return true;
    default:
        // TODO: change this once priming for other firmware types is implemented
        return false;
    }
}

void GCodeExport::writeDelay(const Duration& time_amount)
{
    *output_stream_ << "G4 P" << int(time_amount * 1000) << new_line_;
    estimate_calculator_.addTime(time_amount);
}

void GCodeExport::writeTravel(const Point2LL& p, const Velocity& speed)
{
    writeTravel(Point3LL(p.X, p.Y, current_layer_z_), speed);
}
void GCodeExport::writeExtrusion(const Point2LL& p, const Velocity& speed, double extrusion_mm3_per_mm, PrintFeatureType feature, bool update_extrusion_offset)
{
    writeExtrusion(Point3LL(p.X, p.Y, current_layer_z_), speed, extrusion_mm3_per_mm, feature, update_extrusion_offset);
}

void GCodeExport::writeTravel(const Point3LL& p, const Velocity& speed)
{
    if (flavor_ == EGCodeFlavor::BFB)
    {
        writeMoveBFB(p.x_, p.y_, p.z_ + is_z_hopped_, speed, 0.0, PrintFeatureType::MoveCombing);
        return;
    }
    writeTravel(p.x_, p.y_, p.z_ + is_z_hopped_, speed);
}

void GCodeExport::writeApproachToSeam(
    const Point2LL& pos,
    const Velocity& speed,
    const std::vector<SliceLayerPart>& current_mesh_parts,
    const coord_t wall_line_width,
    const coord_t z_seam_approach_inset)
{
    Point2LL current_position{ current_position_.x_, current_position_.y_ };

    if (pos != current_position)
    {
        // Find current model part
        const SliceLayerPart* current_part = nullptr;
        for (const SliceLayerPart& slice_layer_part : current_mesh_parts)
        {
            if (! current_part && slice_layer_part.outline.inside(pos, true))
            {
                current_part = &slice_layer_part;
            }
        }

        if (! current_part)
        {
            spdlog::error("Unable to find associated mesh layer part");
            assert(false && "Unable to find associated mesh layer part");
            std::exit(1);
        }

        coord_t inset = wall_line_width / 2; // Move to the middle of the wall extrusion line
        inset += std::max(z_seam_approach_inset, 10_mu); // Move inside the model, with a minimum value because 0 may cause unwanted intersections
        Shape model_outline = current_part->outline.offset(-inset);

        // Compute required unretraction duration
        const double total_unretraction_amount
            = eToMm(extruder_attr_[current_extruder_].retraction_e_amount_current_) + mm3ToMm(extruder_attr_[current_extruder_].prime_volume_); // in mm
        const double unretraction_speed = extruder_attr_[current_extruder_].last_retraction_prime_speed_; // in mm/s
        const double total_unretraction_duration = total_unretraction_amount / unretraction_speed; // in seconds

        // Compute straight approach vector
        const Point2LL approach_vector = pos - current_position; // in µm
        const coord_t approach_distance = vSize(approach_vector); // in µm
        const double approach_duration = INT2MM(approach_distance) / speed; // in seconds
        const Point2D approach_direction_vector = Point2D(approach_vector).normalized(); // unitary vector, no unit

        // If true, we have enough time to unretract during the approach, split it into one travel move and one unretraction move.
        // If false, We don't have enough time to unretract during the approach, so first make a partial static unretract, then do the full move while unretracting
        bool approach_longer_than_unretract = approach_duration > total_unretraction_duration;
        coord_t unretraction_path_length; // in µm

        if (approach_longer_than_unretract)
        {
            unretraction_path_length = MM2INT(total_unretraction_duration * speed);
        }
        else
        {
            unretraction_path_length = approach_distance;
        }

        // Generate best-case scenario full straight segment
        const Point2LL full_straight_vector = (approach_direction_vector * unretraction_path_length).toPoint2LL(); // in µm
        const OpenPolyline full_straight_segment({ pos - full_straight_vector, pos }); // in µm
        OpenPolyline unretract_move({ pos }); // in µm

        // Compute intersections of the full straight segment with the model
        struct ModelIntersection
        {
            coord_t distance2_to_pos; // in µm2
            Point2LL intersection; // in µm
            const Polygon* intersected_polygon;
            Polyline::const_segments_iterator segment_iterator;
        };

        std::vector<ModelIntersection> intersections;
        for (const Polygon& polygon : model_outline)
        {
            for (auto segment_it = polygon.beginSegments(); segment_it != polygon.endSegments(); ++segment_it)
            {
                std::optional<Point2LL> intersection // in µm
                    = LinearAlg2D::segmentSegmentIntersection(full_straight_segment[0], full_straight_segment[1], (*segment_it).start, (*segment_it).end);
                if (intersection.has_value())
                {
                    const coord_t distance2_to_pos = vSize2(pos - intersection.value()); // in µm2
                    intersections.emplace_back(distance2_to_pos, intersection.value(), &polygon, segment_it);
                }
            }
        }

        // Now interpret the intersections: as the model outline has been offsetted inwards, the target position should now be outside the outline.
        if (intersections.size() == 1)
        {
            // 1 intersection means the segment comes once inside the model and doesn't get out => that's the best case scenario, keep it as is
            unretract_move = full_straight_segment;
        }
        else
        {
            const Polygon* walk_around_polygon{ nullptr };
            Polyline::const_segments_iterator segment_iterator;
            if (! intersections.empty())
            {
                // 2+ intersection means the segment comes inside the model but later gets out => keep the first part and generate the rest by following the external wall
                // Sort intersection by closeness to the target position: the first in the list is the closest
                std::sort(
                    intersections.begin(),
                    intersections.end(),
                    [](const ModelIntersection& intersection1, const ModelIntersection& intersection2)
                    {
                        return intersection1.distance2_to_pos < intersection2.distance2_to_pos;
                    });

                unretract_move.insert(unretract_move.begin(), intersections[1].intersection);
                walk_around_polygon = intersections[1].intersected_polygon;
                segment_iterator = intersections[1].segment_iterator;
            }
            else
            {
                // 0 intersection means the full segment is outside the model => We have to create a path inside by following the external wall
                ClosestPointPolygon closest_point = PolygonUtils::findClosest(pos, model_outline);
                if (closest_point.isValid())
                {
                    walk_around_polygon = closest_point.poly_;
                    segment_iterator = walk_around_polygon->loopOverSegments(
                        walk_around_polygon->beginSegments(),
                        static_cast<Polyline::const_segments_iterator::difference_type>(closest_point.point_idx_) + 2);
                }
            }

            if (walk_around_polygon)
            {
                do
                {
                    const coord_t current_unretraction_path_length = unretract_move.length(); // in µm
                    Point2LL new_segment = unretract_move.front() - (*segment_iterator).start; // in µm
                    const coord_t new_segment_length = vSize(new_segment); // in µm
                    const coord_t new_retraction_path_length = current_unretraction_path_length + new_segment_length; // in µm

                    if (new_retraction_path_length > unretraction_path_length)
                    {
                        // Shorten segment
                        new_segment = vResize(new_segment, unretraction_path_length - current_unretraction_path_length);
                    }

                    unretract_move.insert(unretract_move.begin(), unretract_move.front() - new_segment);

                    segment_iterator = walk_around_polygon->loopOverSegments(segment_iterator, 1);
                } while (unretract_move.length() < unretraction_path_length);
            }
        }

        // Whatever comes after, travel at print speed to start of unretract move
        if (vSize2(unretract_move.front() - current_position_) > (1000 * 1000))
        {
            writeTravel(unretract_move.front(), speed);
        }

        const coord_t actual_unretraction_path_length = unretract_move.length(); // in µm
        const double actual_unretraction_duration = INT2MM(actual_unretraction_path_length) / speed; // in seconds
        const double actual_unretracted_amount = actual_unretraction_duration * unretraction_speed; // in mm
        const double missing_unretraction_amount = total_unretraction_amount - actual_unretracted_amount; // in mm

        if (missing_unretraction_amount > 0.001)
        {
            // Unretraction path is too short for some reason, first make a partial static unretract
            writeUnretractionAndPrime({}, {}, mmToE(missing_unretraction_amount));
        }

        // Now process the different parts of the unretraction move
        for (auto it = unretract_move.beginSegments(); it != unretract_move.endSegments(); ++it)
        {
            double unretraction_factor = static_cast<double>(vSize((*it).end - (*it).start)) / static_cast<double>(actual_unretraction_path_length);
            double unretraction_amount_segment = unretraction_factor * actual_unretracted_amount;
            writeUnretractionAndPrime((*it).end, speed, mmToE(unretraction_amount_segment));
        }
    }
}

void GCodeExport::writeExtrusion(const Point3LL& p, const Velocity& speed, double extrusion_mm3_per_mm, PrintFeatureType feature, bool update_extrusion_offset)
{
    if (flavor_ == EGCodeFlavor::BFB)
    {
        writeMoveBFB(p.x_, p.y_, p.z_, speed, extrusion_mm3_per_mm, feature);
        return;
    }
    writeExtrusion(p.x_, p.y_, p.z_, speed, extrusion_mm3_per_mm, feature, update_extrusion_offset);
}

void GCodeExport::writeMoveBFB(const int x, const int y, const int z, const Velocity& speed, double extrusion_mm3_per_mm, PrintFeatureType feature)
{
    if (std::isinf(extrusion_mm3_per_mm))
    {
        spdlog::error("Extrusion rate is infinite!");
        assert(false && "Infinite extrusion move!");
        std::exit(1);
    }
    if (std::isnan(extrusion_mm3_per_mm))
    {
        spdlog::error("Extrusion rate is not a number!");
        assert(false && "NaN extrusion move!");
        std::exit(1);
    }

    double extrusion_per_mm = mm3ToE(extrusion_mm3_per_mm);

    // For Bits From Bytes machines, we need to handle this completely differently. As they do not use E values but RPM values.
    double fspeed = speed * 60;
    double rpm = extrusion_per_mm * speed * 60;
    const double mm_per_rpm = 4.0; // All BFB machines have 4mm per RPM extrusion.
    rpm /= mm_per_rpm;
    if (rpm > 0)
    {
        if (extruder_attr_[current_extruder_].retraction_e_amount_current_)
        {
            if (current_speed_ != double(rpm))
            {
                // fprintf(f, "; %f e-per-mm %d mm-width %d mm/s\n", extrusion_per_mm, lineWidth, speed);
                // fprintf(f, "M108 S%0.1f\r\n", rpm);
                *output_stream_ << "M108 S" << PrecisionedDouble{ 1, rpm } << new_line_;
                current_speed_ = double(rpm);
            }
            // Add M101 or M201 to enable the proper extruder.
            *output_stream_ << "M" << int((current_extruder_ + 1) * 100 + 1) << new_line_;
            extruder_attr_[current_extruder_].retraction_e_amount_current_ = 0.0;
        }
        // Fix the speed by the actual RPM we are asking, because of rounding errors we cannot get all RPM values, but we have a lot more resolution in the feedrate value.
        //  (Trick copied from KISSlicer, thanks Jonathan)
        fspeed *= (rpm / (roundf(rpm * 100) / 100));

        // Increase the extrusion amount to calculate the amount of filament used.
        Point3LL diff = Point3LL(x, y, z) - getPosition();

        current_e_value_ += extrusion_per_mm * diff.vSizeMM();
    }
    else
    {
        // If we are not extruding, check if we still need to disable the extruder. This causes a retraction due to auto-retraction.
        if (! extruder_attr_[current_extruder_].retraction_e_amount_current_)
        {
            *output_stream_ << "M103" << new_line_;
            extruder_attr_[current_extruder_].retraction_e_amount_current_
                = 1.0; // 1.0 used as stub; BFB doesn't use the actual retraction amount; it performs retraction on the firmware automatically
        }
    }

    writeGCommand("G1", fspeed, x, y, z, 0.0, feature);
}

void GCodeExport::writeTravel(const coord_t x, const coord_t y, const coord_t z, const Velocity& speed)
{
    if (current_position_.x_ == x && current_position_.y_ == y && current_position_.z_ == z)
    {
        return;
    }

#ifdef ASSERT_INSANE_OUTPUT
    assert(speed < 1000 && speed > 1); // normal F values occurring in UM2 gcode (this code should not be compiled for release)
    assert(current_position_ != no_point3);
    assert(Point3LL(x, y, z) != no_point3);
    assert((Point3LL(x, y, z) - current_position_).vSize() < MM2INT(1000)); // no crazy positions (this code should not be compiled for release)
#endif // ASSERT_INSANE_OUTPUT

    const PrintFeatureType travel_move_type = extruder_attr_[current_extruder_].retraction_e_amount_current_ ? PrintFeatureType::MoveRetraction : PrintFeatureType::MoveCombing;
    sendTravelLine(Point2LL(x, y), travel_move_type, speed);

    writeGCommand("G0", speed, x, y, z, 0.0, travel_move_type);
}

void GCodeExport::writeExtrusion(
    const coord_t x,
    const coord_t y,
    const coord_t z,
    const Velocity& speed,
    const double extrusion_mm3_per_mm,
    const PrintFeatureType& feature,
    const bool update_extrusion_offset)
{
    if (current_position_.x_ == x && current_position_.y_ == y && current_position_.z_ == z)
    {
        return;
    }

#ifdef ASSERT_INSANE_OUTPUT
    assert(speed < 1000 && speed > 1); // normal F values occurring in UM2 gcode (this code should not be compiled for release)
    assert(current_position_ != no_point3);
    assert(Point3LL(x, y, z) != no_point3);
    assert((Point3LL(x, y, z) - current_position_).vSize() < MM2INT(1000)); // no crazy positions (this code should not be compiled for release)
    assert(extrusion_mm3_per_mm >= 0.0);
#endif // ASSERT_INSANE_OUTPUT
#ifdef DEBUG
    if (std::isinf(extrusion_mm3_per_mm))
    {
        spdlog::error("Extrusion rate is infinite!");
        assert(false && "Infinite extrusion move!");
        std::exit(1);
    }

    if (std::isnan(extrusion_mm3_per_mm))
    {
        spdlog::error("Extrusion rate is not a number!");
        assert(false && "NaN extrusion move!");
        std::exit(1);
    }

    if (extrusion_mm3_per_mm < 0.0)
    {
        spdlog::warn("Warning! Negative extrusion move!\n");
    }
#endif

    const double extrusion_per_mm = mm3ToE(extrusion_mm3_per_mm);

    if (is_z_hopped_ > 0)
    {
        writeZhopEnd();
    }

    const Point3LL diff = Point3LL(x, y, z) - current_position_;
    const double diff_length = diff.vSizeMM();

    writeUnretractionAndPrime();

    // flow rate compensation
    double extrusion_offset = 0;
    if (diff_length)
    {
        extrusion_offset = speed * extrusion_mm3_per_mm * extrusion_offset_factor_;
        if (extrusion_offset > max_extrusion_offset_)
        {
            extrusion_offset = max_extrusion_offset_;
        }
    }
    // write new value of extrusion_offset, which will be remembered.
    if (update_extrusion_offset && (extrusion_offset != current_e_offset_))
    {
        current_e_offset_ = extrusion_offset;
        *output_stream_ << ";FLOW_RATE_COMPENSATED_OFFSET = " << current_e_offset_ << new_line_;
    }

    extruder_attr_[current_extruder_].last_e_value_after_wipe_ += extrusion_per_mm * diff_length;
    const double e_delta = extrusion_per_mm * diff_length;

    writeGCommand("G1", speed, x, y, z, e_delta, feature);
}

void GCodeExport::writeGCommand(
    const char* command_name,
    const Velocity& speed,
    const coord_t x,
    const coord_t y,
    const coord_t z,
    const double e_delta,
    const PrintFeatureType feature)
{
    const Point2LL gcode_pos = getGcodePos(x, y, current_extruder_);
    total_bounding_box_.include(Point3LL(gcode_pos.X, gcode_pos.Y, z));

    const double actual_e_delta = e_delta + current_e_offset_;
    const bool add_speed = current_speed_ != speed;
    const bool add_x = gcode_pos.X != current_position_.x_;
    const bool add_y = gcode_pos.Y != current_position_.y_;
    const bool add_z = z != current_position_.z_;
    const bool add_e = std::fabs(actual_e_delta) >= 0.00001;

    if (add_speed || add_x || add_y || add_z || add_e)
    {
        *output_stream_ << command_name;

        if (add_speed)
        {
            *output_stream_ << " F" << PrecisionedDouble{ 1, speed * 60 };
            current_speed_ = speed;
        }

        if (add_x)
        {
            *output_stream_ << " X" << MMtoStream{ gcode_pos.X };
            current_position_.x_ = x;
        }
        if (add_y)
        {
            *output_stream_ << " Y" << MMtoStream{ gcode_pos.Y };
            current_position_.y_ = y;
        }
        if (add_z)
        {
            *output_stream_ << " Z" << MMtoStream{ z };
            current_position_.z_ = z;
        }

        if (add_e)
        {
            current_e_value_ += actual_e_delta;
            const double output_e = (relative_extrusion_) ? actual_e_delta : current_e_value_;
            *output_stream_ << " " << extruder_attr_[current_extruder_].extruder_character_ << PrecisionedDouble{ 5, output_e };
        }

        *output_stream_ << new_line_;

        estimate_calculator_.plan(TimeEstimateCalculator::Position(INT2MM(x), INT2MM(y), INT2MM(z), eToMm(current_e_value_)), speed, feature);
    }
}

void GCodeExport::writeUnretractionAndPrime(const std::optional<Point2LL>& pos, const std::optional<Velocity>& override_speed, const std::optional<double>& override_amount)
{
    const double prime_volume = extruder_attr_[current_extruder_].prime_volume_;
    const double prime_volume_e = mm3ToE(prime_volume);
    double unretract_e = extruder_attr_[current_extruder_].retraction_e_amount_current_;
    const Point2LL actual_pos = pos.value_or(Point2LL(current_position_.x_, current_position_.y_));

    if (extruder_attr_[current_extruder_].machine_firmware_retract_)
    { // note that BFB unretract is handled differently
        *output_stream_ << "G11" << new_line_;
        current_e_value_ += unretract_e;
        unretract_e = 0.0;

        // Assume default UM2 retraction settings.
        estimate_calculator_.plan(
            TimeEstimateCalculator::Position(INT2MM(current_position_.x_), INT2MM(current_position_.y_), INT2MM(current_position_.z_), eToMm(current_e_value_)),
            25.0,
            PrintFeatureType::MoveUnretraction);
    }

    const double total_remaining_unretract = unretract_e + prime_volume_e;
    const double actual_total_remaining_unretract = override_amount.value_or(total_remaining_unretract);

    if (actual_total_remaining_unretract != 0.0)
    {
        Velocity actual_speed = override_speed.value_or(extruder_attr_[current_extruder_].last_retraction_prime_speed_);

        PrintFeatureType line_type = unretract_e != 0.0 ? PrintFeatureType::MoveUnretraction : PrintFeatureType::NoneType;
        writeGCommand("G1", actual_speed, actual_pos.X, actual_pos.Y, current_position_.z_, actual_total_remaining_unretract, line_type);

        if (pos.has_value())
        {
            sendTravelLine(pos.value(), line_type, actual_speed);
        }
    }

    extruder_attr_[current_extruder_].prime_volume_ = 0.0;

    if (override_amount.has_value())
    {
        extruder_attr_[current_extruder_].retraction_e_amount_current_ = total_remaining_unretract - override_amount.value();
    }
    else
    {
        if (getCurrentExtrudedVolume() > 10000.0 && flavor_ != EGCodeFlavor::BFB
            && flavor_ != EGCodeFlavor::MAKERBOT) // According to https://github.com/Ultimaker/CuraEngine/issues/14 having more then 21m of extrusion causes inaccuracies. So reset
                                                  // it every 10m, just to be sure.
        {
            resetExtrusionValue();
        }

        extruder_attr_[current_extruder_].retraction_e_amount_current_ = 0.0;
    }
}

void GCodeExport::writeRetraction(const RetractionConfig& config, bool force, bool extruder_switch)
{
    ExtruderTrainAttributes& extr_attr = extruder_attr_[current_extruder_];

    if (flavor_ == EGCodeFlavor::BFB) // BitsFromBytes does automatic retraction.
    {
        if (extruder_switch)
        {
            if (! extr_attr.retraction_e_amount_current_)
            {
                *output_stream_ << "M103" << new_line_;
            }
            extr_attr.retraction_e_amount_current_ = 1.0; // 1.0 is a stub; BFB doesn't use the actual retracted amount; retraction is performed by firmware
        }
        return;
    }

    double old_retraction_e_amount = extr_attr.retraction_e_amount_current_;
    double new_retraction_e_amount = mmToE(config.distance);
    double retraction_diff_e_amount = old_retraction_e_amount - new_retraction_e_amount;
    if (std::abs(retraction_diff_e_amount) < 0.000001)
    {
        return;
    }

    { // handle retraction limitation
        double current_extruded_volume = getCurrentExtrudedVolume();
        std::deque<double>& extruded_volume_at_previous_n_retractions = extr_attr.extruded_volume_at_previous_n_retractions_;
        while (extruded_volume_at_previous_n_retractions.size() > config.retraction_count_max && ! extruded_volume_at_previous_n_retractions.empty())
        {
            // extruder switch could have introduced data which falls outside the retraction window
            // also the retraction_count_max could have changed between the last retraction and this
            extruded_volume_at_previous_n_retractions.pop_back();
        }
        if (! force && config.retraction_count_max <= 0)
        {
            return;
        }
        if (! force && extruded_volume_at_previous_n_retractions.size() == config.retraction_count_max
            && current_extruded_volume < extruded_volume_at_previous_n_retractions.back() + config.retraction_extrusion_window * extr_attr.filament_area_)
        {
            return;
        }
        extruded_volume_at_previous_n_retractions.push_front(current_extruded_volume);
        if (extruded_volume_at_previous_n_retractions.size() == config.retraction_count_max + 1)
        {
            extruded_volume_at_previous_n_retractions.pop_back();
        }
    }

    if (extr_attr.machine_firmware_retract_)
    {
        if (extruder_switch && extr_attr.retraction_e_amount_current_)
        {
            return;
        }
        *output_stream_ << "G10";
        if (extruder_switch && flavor_ == EGCodeFlavor::REPETIER)
        {
            *output_stream_ << " S1";
        }
        *output_stream_ << new_line_;
        // Assume default UM2 retraction settings.
        estimate_calculator_.plan(
            TimeEstimateCalculator::Position(
                INT2MM(current_position_.x_),
                INT2MM(current_position_.y_),
                INT2MM(current_position_.z_),
                eToMm(current_e_value_ + retraction_diff_e_amount)),
            25.0,
            PrintFeatureType::MoveRetraction); // TODO: hardcoded values!
    }
    else
    {
        double speed = ((retraction_diff_e_amount < 0.0) ? config.speed : extr_attr.last_retraction_prime_speed_);
        extr_attr.last_retraction_prime_speed_ = config.primeSpeed;

        writeGCommand("G1", speed, current_position_.x_, current_position_.y_, current_position_.z_, retraction_diff_e_amount, PrintFeatureType::MoveRetraction);
    }

    extr_attr.retraction_e_amount_current_ = new_retraction_e_amount; // suppose that for UM2 the retraction amount in the firmware is equal to the provided amount
    extr_attr.prime_volume_ += config.prime_volume;
}

void GCodeExport::writeZhopStart(const coord_t hop_height, Velocity speed /*= 0*/)
{
    if (hop_height > 0)
    {
        if (speed == 0)
        {
            const ExtruderTrain& extruder = Application::getInstance().current_slice_->scene.extruders[current_extruder_];
            speed = extruder.settings_.get<Velocity>("speed_z_hop");
        }
        is_z_hopped_ = hop_height;
        current_speed_ = speed;
        *output_stream_ << "G1 F" << PrecisionedDouble{ 1, speed * 60 } << " Z" << MMtoStream{ current_layer_z_ + is_z_hopped_ } << new_line_;
        total_bounding_box_.includeZ(current_layer_z_ + is_z_hopped_);
        assert(speed > 0.0 && "Z hop speed should be positive.");
    }
}

void GCodeExport::writeZhopEnd(Velocity speed /*= 0*/)
{
    if (is_z_hopped_)
    {
        if (speed == 0)
        {
            const ExtruderTrain& extruder = Application::getInstance().current_slice_->scene.extruders[current_extruder_];
            speed = extruder.settings_.get<Velocity>("speed_z_hop");
        }
        is_z_hopped_ = 0;
        current_position_.z_ = current_layer_z_;
        current_speed_ = speed;
        *output_stream_ << "G1 F" << PrecisionedDouble{ 1, speed * 60 } << " Z" << MMtoStream{ current_layer_z_ } << new_line_;
        assert(speed > 0.0 && "Z hop speed should be positive.");
    }
}

void GCodeExport::startExtruder(const size_t new_extruder)
{
    extruder_attr_[new_extruder].is_used_ = true;
    if (new_extruder != current_extruder_) // wouldn't be the case on the very first extruder start if it's extruder 0
    {
        if (flavor_ == EGCodeFlavor::MAKERBOT)
        {
            *output_stream_ << "M135 T" << new_extruder << new_line_;
        }
        else
        {
            *output_stream_ << "T" << new_extruder << new_line_;
        }
    }

    current_extruder_ = new_extruder;

    assert(getCurrentExtrudedVolume() == 0.0 && "Just after an extruder switch we haven't extruded anything yet!");
    resetExtrusionValue(); // zero the E value on the new extruder, just to be sure

    const auto extruder_settings = Application::getInstance().current_slice_->scene.extruders[new_extruder].settings_;
    const auto start_code = extruder_settings.get<std::string>("machine_extruder_start_code");
    if (! start_code.empty())
    {
        if (relative_extrusion_)
        {
            writeExtrusionMode(false); // ensure absolute extrusion mode is set before the start gcode
        }

        writeCode(start_code.c_str());

        if (relative_extrusion_)
        {
            writeExtrusionMode(true); // restore relative extrusion mode
        }
    }

    const auto start_code_duration = extruder_settings.get<Duration>("machine_extruder_start_code_duration");
    estimate_calculator_.addTime(start_code_duration);

    Application::getInstance().communication_->setExtruderForSend(Application::getInstance().current_slice_->scene.extruders[new_extruder]);
    Application::getInstance().communication_->sendCurrentPosition(getPositionXY());

    // Change the Z position so it gets re-written again. We do not know if the switch code modified the Z position.
    current_position_.z_ += 1;

    setExtruderFanNumber(new_extruder);
}

void GCodeExport::switchExtruder(size_t new_extruder, const RetractionConfig& retraction_config_old_extruder, coord_t perform_z_hop /*= 0*/)
{
    if (current_extruder_ == new_extruder)
    {
        return;
    }

    const Settings& old_extruder_settings = Application::getInstance().current_slice_->scene.extruders[current_extruder_].settings_;
    if (old_extruder_settings.get<bool>("retraction_enable"))
    {
        constexpr bool force = true;
        constexpr bool extruder_switch = true;
        writeRetraction(retraction_config_old_extruder, force, extruder_switch);
    }

    if (perform_z_hop > 0)
    {
        writeZhopStart(perform_z_hop);
    }

    resetExtrusionValue(); // zero the E value on the old extruder, so that the current_e_value is registered on the old extruder

    const auto end_code = old_extruder_settings.get<std::string>("machine_extruder_end_code");

    if (! end_code.empty())
    {
        if (relative_extrusion_)
        {
            writeExtrusionMode(false); // ensure absolute extrusion mode is set before the end gcode
        }

        writeCode(end_code.c_str());

        if (relative_extrusion_)
        {
            writeExtrusionMode(true); // restore relative extrusion mode
        }
    }

    const auto end_code_duration = old_extruder_settings.get<Duration>("machine_extruder_end_code_duration");
    estimate_calculator_.addTime(end_code_duration);

    startExtruder(new_extruder);
}

void GCodeExport::writeCode(const char* str)
{
    *output_stream_ << str << new_line_;
}

void GCodeExport::resetExtruderToPrimed(const size_t extruder, const double initial_retraction)
{
    extruder_attr_[extruder].is_primed_ = true;

    extruder_attr_[extruder].retraction_e_amount_current_ = initial_retraction;
}

void GCodeExport::writePrimeTrain(const Velocity& travel_speed)
{
    if (extruder_attr_[current_extruder_].is_primed_)
    { // extruder is already primed once!
        return;
    }
    const Settings& extruder_settings = Application::getInstance().current_slice_->scene.extruders[current_extruder_].settings_;
    if (extruder_settings.get<bool>("prime_blob_enable"))
    { // only move to prime position if we do a blob/poop
        // ideally the prime position would be respected whether we do a blob or not,
        // but the frontend currently doesn't support a value function of an extruder setting depending on an fdmprinter setting,
        // which is needed to automatically ignore the prime position for the printer when blob is disabled
        Point3LL prime_pos(
            extruder_settings.get<coord_t>("extruder_prime_pos_x"),
            extruder_settings.get<coord_t>("extruder_prime_pos_y"),
            extruder_settings.get<coord_t>("extruder_prime_pos_z"));
        if (! extruder_settings.get<bool>("extruder_prime_pos_abs"))
        {
            // currentPosition.z can be already z hopped
            prime_pos += Point3LL(current_position_.x_, current_position_.y_, current_layer_z_);
        }
        writeTravel(prime_pos, travel_speed);
    }

    if (flavor_ == EGCodeFlavor::GRIFFIN)
    {
        bool should_correct_z = false;

        std::string command = "G280";
        if (! extruder_settings.get<bool>("prime_blob_enable"))
        {
            command += " S1"; // use S1 to disable prime blob
            should_correct_z = true;
        }
        *output_stream_ << command << new_line_;

        // There was an issue with the S1 strategy parameter, where it would only change the material-position,
        //   as opposed to 'be a prime-blob maneuvre without actually printing the prime blob', as we assumed here.
        // After a chat, the firmware-team decided to change the S1 strategy behaviour,
        //   but since people don't update their firmware at each opportunity, it was decided to fix it here as well.
        if (should_correct_z)
        {
            // Can't output via 'writeTravel', since if this is needed, the value saved for 'current height' will not be correct.
            // For similar reasons, this isn't written to the front-end via command-socket.
            *output_stream_ << "G0 Z" << MMtoStream{ getPositionZ() } << new_line_;
        }
    }
    else
    {
        // there is no prime gcode for other firmware versions...
    }

    extruder_attr_[current_extruder_].is_primed_ = true;
}

void GCodeExport::setExtruderFanNumber(int extruder)
{
    if (extruder_attr_[extruder].fan_number_ != fan_number_)
    {
        fan_number_ = extruder_attr_[extruder].fan_number_;
        current_fan_speed_ = -1; // ensure fan speed gcode gets output for this fan
    }
}

void GCodeExport::writeFanCommand(double speed)
{
    if (std::abs(current_fan_speed_ - speed) < 0.1)
    {
        return;
    }
    if (flavor_ == EGCodeFlavor::MAKERBOT)
    {
        if (speed >= 50)
        {
            *output_stream_ << "M126 T0" << new_line_; // Makerbot cannot PWM the fan speed...
        }
        else
        {
            *output_stream_ << "M127 T0" << new_line_;
        }
    }
    else if (speed > 0)
    {
        const bool should_scale_zero_to_one = Application::getInstance().current_slice_->scene.settings.get<bool>("machine_scale_fan_speed_zero_to_one");
        *output_stream_ << "M106 S"
                        << PrecisionedDouble{ (should_scale_zero_to_one ? static_cast<uint8_t>(2) : static_cast<uint8_t>(1)),
                                              (should_scale_zero_to_one ? speed : speed * 255) / 100 };
        if (fan_number_)
        {
            *output_stream_ << " P" << fan_number_;
        }
        *output_stream_ << new_line_;
    }
    else
    {
        *output_stream_ << "M107";
        if (fan_number_)
        {
            *output_stream_ << " P" << fan_number_;
        }
        *output_stream_ << new_line_;
    }

    current_fan_speed_ = speed;
}

void GCodeExport::writeTemperatureCommand(const size_t extruder, const Temperature& temperature, const bool wait)
{
    const ExtruderTrain& extruder_train = Application::getInstance().current_slice_->scene.extruders[extruder];

    if (! extruder_train.settings_.get<bool>("machine_nozzle_temp_enabled"))
    {
        return;
    }

    if (extruder_train.settings_.get<bool>("machine_extruders_share_heater"))
    {
        // extruders share a single heater
        if (extruder != current_extruder_)
        {
            // ignore all changes to the non-current extruder
            return;
        }

        // sync all extruders with the change to the current extruder
        const size_t extruder_count = Application::getInstance().current_slice_->scene.extruders.size();

        for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
        {
            if (extruder_nr != extruder)
            {
                // only reset the other extruders' waited_for_temperature state when the new temperature
                // is greater than the old temperature
                if (wait || temperature > extruder_attr_[extruder_nr].current_temperature_)
                {
                    extruder_attr_[extruder_nr].waited_for_temperature_ = wait;
                }
                extruder_attr_[extruder_nr].current_temperature_ = temperature;
            }
        }
    }

    if ((! wait || extruder_attr_[extruder].waited_for_temperature_) && extruder_attr_[extruder].current_temperature_ == temperature)
    {
        return;
    }

    if (wait && flavor_ != EGCodeFlavor::MAKERBOT)
    {
        if (flavor_ == EGCodeFlavor::MARLIN)
        {
            *output_stream_ << "M105" << new_line_; // get temperatures from the last update, the M109 will not let get the target temperature
        }
        *output_stream_ << "M109";
        extruder_attr_[extruder].waited_for_temperature_ = true;
    }
    else
    {
        *output_stream_ << "M104";
        extruder_attr_[extruder].waited_for_temperature_ = false;
    }
    if (extruder != current_extruder_)
    {
        *output_stream_ << " T" << extruder;
    }
#ifdef ASSERT_INSANE_OUTPUT
    assert(temperature >= 0);
#endif // ASSERT_INSANE_OUTPUT
    *output_stream_ << " S" << PrecisionedDouble{ 1, temperature } << new_line_;
    if (extruder != current_extruder_ && always_write_active_tool_)
    {
        // Some firmwares (ie Smoothieware) change tools every time a "T" command is read - even on a M104 line, so we need to switch back to the active tool.
        *output_stream_ << "T" << current_extruder_ << new_line_;
    }
    if (wait && flavor_ == EGCodeFlavor::MAKERBOT)
    {
        // Makerbot doesn't use M109 for heat-and-wait. Instead, use M104 and then wait using M116.
        *output_stream_ << "M116" << new_line_;
    }
    extruder_attr_[extruder].current_temperature_ = temperature;
}

void GCodeExport::writeBedTemperatureCommand(const Temperature& temperature, const bool wait)
{
    if (flavor_ == EGCodeFlavor::ULTIGCODE)
    { // The UM2 family doesn't support temperature commands (they are fixed in the firmware)
        return;
    }

    if (bed_temperature_ != temperature) // Not already at the desired temperature.
    {
        if (wait)
        {
            if (flavor_ == EGCodeFlavor::MARLIN)
            {
                *output_stream_ << "M140 S"; // set the temperature, it will be used as target temperature from M105
                *output_stream_ << PrecisionedDouble{ 1, temperature } << new_line_;
                *output_stream_ << "M105" << new_line_;
            }
            *output_stream_ << "M190 S";
        }
        else
        {
            *output_stream_ << "M140 S";
        }

        *output_stream_ << PrecisionedDouble{ 1, temperature } << new_line_;

        bed_temperature_ = temperature;
    }
}

void GCodeExport::writeBuildVolumeTemperatureCommand(const Temperature& temperature, const bool wait)
{
    if (flavor_ == EGCodeFlavor::ULTIGCODE || flavor_ == EGCodeFlavor::GRIFFIN)
    {
        // Ultimaker printers don't support build volume temperature commands.
        return;
    }
    if (wait)
    {
        *output_stream_ << "M191 S";
    }
    else
    {
        *output_stream_ << "M141 S";
    }
    *output_stream_ << PrecisionedDouble{ 1, temperature } << new_line_;
}

void GCodeExport::writePrintAcceleration(const Acceleration& acceleration)
{
    switch (getFlavor())
    {
    case EGCodeFlavor::REPETIER:
        if (current_print_acceleration_ != acceleration)
        {
            *output_stream_ << "M201 X" << PrecisionedDouble{ 0, acceleration } << " Y" << PrecisionedDouble{ 0, acceleration } << new_line_;
        }
        break;
    case EGCodeFlavor::REPRAP:
        if (current_print_acceleration_ != acceleration)
        {
            *output_stream_ << "M204 P" << PrecisionedDouble{ 0, acceleration } << new_line_;
        }
        break;
    default:
        // MARLIN, etc. only have one acceleration for both print and travel
        if (current_print_acceleration_ != acceleration)
        {
            *output_stream_ << "M204 S" << PrecisionedDouble{ 0, acceleration } << new_line_;
        }
        break;
    }
    current_print_acceleration_ = acceleration;
    estimate_calculator_.setAcceleration(acceleration);
}

void GCodeExport::writeTravelAcceleration(const Acceleration& acceleration)
{
    switch (getFlavor())
    {
    case EGCodeFlavor::REPETIER:
        if (current_travel_acceleration_ != acceleration)
        {
            *output_stream_ << "M202 X" << PrecisionedDouble{ 0, acceleration } << " Y" << PrecisionedDouble{ 0, acceleration } << new_line_;
        }
        break;
    case EGCodeFlavor::REPRAP:
        if (current_travel_acceleration_ != acceleration)
        {
            *output_stream_ << "M204 T" << PrecisionedDouble{ 0, acceleration } << new_line_;
        }
        break;
    default:
        // MARLIN, etc. only have one acceleration for both print and travel
        writePrintAcceleration(acceleration);
        break;
    }
    current_travel_acceleration_ = acceleration;
    estimate_calculator_.setAcceleration(acceleration);
}

void GCodeExport::writeJerk(const Velocity& jerk)
{
    if (current_jerk_ != jerk)
    {
        switch (getFlavor())
        {
        case EGCodeFlavor::REPETIER:
            *output_stream_ << "M207 X" << PrecisionedDouble{ 2, jerk } << new_line_;
            break;
        case EGCodeFlavor::REPRAP:
            *output_stream_ << "M566 X" << PrecisionedDouble{ 2, jerk * 60 } << " Y" << PrecisionedDouble{ 2, jerk * 60 } << new_line_;
            break;
        default:
            *output_stream_ << "M205 X" << PrecisionedDouble{ 2, jerk } << " Y" << PrecisionedDouble{ 2, jerk } << new_line_;
            break;
        }
        current_jerk_ = jerk;
        estimate_calculator_.setMaxXyJerk(jerk);
    }
}

void GCodeExport::finalize(const char* endCode)
{
    writeFanCommand(0);
    writeCode(endCode);
    int64_t print_time = getSumTotalPrintTimes();
    int mat_0 = getTotalFilamentUsed(0);
    spdlog::info("Print time (s): {}", print_time);
    spdlog::info("Print time (hr|min|s): {}h {}m {}s", int(print_time / 60 / 60), int((print_time / 60) % 60), int(print_time % 60));
    spdlog::info("Filament (mm^3): {}", mat_0);
    for (int n = 1; n < MAX_EXTRUDERS; n++)
        if (getTotalFilamentUsed(n) > 0)
            spdlog::info("Filament {}: {}", n + 1, int(getTotalFilamentUsed(n)));
    output_stream_->flush();
}

double GCodeExport::getExtrudedVolumeAfterLastWipe(size_t extruder)
{
    return eToMm3(extruder_attr_[extruder].last_e_value_after_wipe_, extruder);
}

void GCodeExport::ResetLastEValueAfterWipe(size_t extruder)
{
    extruder_attr_[extruder].last_e_value_after_wipe_ = 0;
}

void GCodeExport::insertWipeScript(const WipeScriptConfig& wipe_config)
{
    const Point3LL prev_position = current_position_;
    writeComment("WIPE_SCRIPT_BEGIN");

    if (wipe_config.retraction_enable)
    {
        writeRetraction(wipe_config.retraction_config);
    }

    if (wipe_config.hop_enable)
    {
        writeZhopStart(wipe_config.hop_amount, wipe_config.hop_speed);
    }

    writeTravel(Point2LL(wipe_config.brush_pos_x, current_position_.y_), wipe_config.move_speed);
    for (size_t i = 0; i < wipe_config.repeat_count; ++i)
    {
        coord_t x = current_position_.x_ + (i % 2 ? -wipe_config.move_distance : wipe_config.move_distance);
        writeTravel(Point2LL(x, current_position_.y_), wipe_config.move_speed);
    }

    writeTravel(prev_position, wipe_config.move_speed);

    if (wipe_config.hop_enable)
    {
        writeZhopEnd(wipe_config.hop_speed);
    }

    if (wipe_config.retraction_enable)
    {
        writeUnretractionAndPrime();
    }

    if (wipe_config.pause > 0)
    {
        writeDelay(wipe_config.pause);
    }

    writeComment("WIPE_SCRIPT_END");
}

void GCodeExport::setSliceUUID(const std::string& slice_uuid)
{
    slice_uuid_ = slice_uuid;
}

} // namespace cura
