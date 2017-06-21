//Copyright (C) 2013 David Braam
//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <stdarg.h>
#include <iomanip>
#include <cmath>

#include "gcodeExport.h"
#include "utils/logoutput.h"
#include "PrintFeature.h"
#include "utils/Date.h"
#include "utils/string.h" // MMtoStream, PrecisionedDouble

namespace cura {

double layer_height; //!< report basic layer height in RepRap gcode file.

GCodeExport::GCodeExport()
: output_stream(&std::cout)
, currentPosition(0,0,MM2INT(20))
, layer_nr(0)
{
    *output_stream << std::fixed;

    current_e_value = 0;
    current_extruder = 0;
    currentFanSpeed = -1;

    total_print_times = std::vector<double>(static_cast<unsigned char>(PrintFeatureType::NumPrintFeatureTypes), 0.0);

    currentSpeed = 1;
    current_acceleration = -1;
    current_travel_acceleration = -1;
    current_jerk = -1;
    current_max_z_feedrate = -1;

    isZHopped = 0;
    setFlavor(EGCodeFlavor::REPRAP);
    initial_bed_temp = 0;

    extruder_count = 0;

    total_bounding_box = AABB3D();
}

GCodeExport::~GCodeExport()
{
}

void GCodeExport::preSetup(const MeshGroup* meshgroup)
{
    setFlavor(meshgroup->getSettingAsGCodeFlavor("machine_gcode_flavor"));
    use_extruder_offset_to_offset_coords = meshgroup->getSettingBoolean("machine_use_extruder_offset_to_offset_coords");

    extruder_count = meshgroup->getSettingAsCount("machine_extruder_count");

    for (unsigned int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        const ExtruderTrain* train = meshgroup->getExtruderTrain(extruder_nr);
        setFilamentDiameter(extruder_nr, train->getSettingInMicrons("material_diameter")); 

        extruder_attr[extruder_nr].prime_pos = Point3(train->getSettingInMicrons("extruder_prime_pos_x"), train->getSettingInMicrons("extruder_prime_pos_y"), train->getSettingInMicrons("extruder_prime_pos_z"));
        extruder_attr[extruder_nr].prime_pos_is_abs = train->getSettingBoolean("extruder_prime_pos_abs");
        extruder_attr[extruder_nr].use_temp = train->getSettingBoolean("machine_nozzle_temp_enabled");
        extruder_attr[extruder_nr].is_prime_blob_enabled = train->getSettingBoolean("prime_blob_enable");

        extruder_attr[extruder_nr].nozzle_size = train->getSettingInMicrons("machine_nozzle_size");
        extruder_attr[extruder_nr].nozzle_offset = Point(train->getSettingInMicrons("machine_nozzle_offset_x"), train->getSettingInMicrons("machine_nozzle_offset_y"));
        extruder_attr[extruder_nr].material_guid = train->getSettingString("material_guid");

        extruder_attr[extruder_nr].start_code = train->getSettingString("machine_extruder_start_code");
        extruder_attr[extruder_nr].end_code = train->getSettingString("machine_extruder_end_code");

        extruder_attr[extruder_nr].last_retraction_prime_speed = train->getSettingInMillimetersPerSecond("retraction_prime_speed"); // the alternative would be switch_extruder_prime_speed, but dual extrusion might not even be configured...
    }

    machine_name = meshgroup->getSettingString("machine_name");

    layer_height = meshgroup->getSettingInMillimeters("layer_height");

    if (flavor == EGCodeFlavor::BFB)
    {
        new_line = "\r\n";
    }
    else 
    {
        new_line = "\n";
    }

    estimateCalculator.setFirmwareDefaults(meshgroup);
}

void GCodeExport::setInitialTemps(const MeshGroup& settings, const unsigned int start_extruder_nr)
{
    for (unsigned int extr_nr = 0; extr_nr < extruder_count; extr_nr++)
    {
        const ExtruderTrain& train = *settings.getExtruderTrain(extr_nr);
        
        double print_temp_0 = train.getSettingInDegreeCelsius("material_print_temperature_layer_0");
        double print_temp_here = (print_temp_0 != 0)? print_temp_0 : train.getSettingInDegreeCelsius("material_print_temperature");
        double temp = (extr_nr == start_extruder_nr)? print_temp_here : train.getSettingInDegreeCelsius("material_standby_temperature");
        setInitialTemp(extr_nr, temp);
    }

    initial_bed_temp = settings.getSettingInDegreeCelsius("material_bed_temperature_layer_0");
}

void GCodeExport::setInitialTemp(int extruder_nr, double temp)
{
    extruder_attr[extruder_nr].initial_temp = temp;
    if (flavor == EGCodeFlavor::GRIFFIN || flavor == EGCodeFlavor::ULTIGCODE)
    {
        extruder_attr[extruder_nr].currentTemperature = temp;
    }
}


std::string GCodeExport::getFileHeader(const std::vector<bool>& extruder_is_used, const double* print_time, const std::vector<double>& filament_used, const std::vector<std::string>& mat_ids)
{
    std::ostringstream prefix;

    // output common headers
    prefix << ";FLAVOR:" << toString(flavor) << new_line;

    // output flavor specific headers
    switch (flavor)
    {
    case EGCodeFlavor::GRIFFIN:
        prefix << ";START_OF_HEADER" << new_line;
        prefix << ";HEADER_VERSION:0.1" << new_line;
        prefix << ";GENERATOR.NAME:Cura_SteamEngine" << new_line;
        prefix << ";GENERATOR.VERSION:" << VERSION << new_line;
        prefix << ";GENERATOR.BUILD_DATE:" << Date::getDate().toStringDashed() << new_line;
        prefix << ";TARGET_MACHINE.NAME:" << machine_name << new_line;

        for (unsigned int extr_nr = 0; extr_nr < extruder_count; extr_nr++)
        {
            if (!extruder_is_used[extr_nr])
            {
                continue;
            }
            prefix << ";EXTRUDER_TRAIN." << extr_nr << ".INITIAL_TEMPERATURE:" << extruder_attr[extr_nr].initial_temp << new_line;
            if (filament_used.size() == extruder_count)
            {
                prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.VOLUME_USED:" << static_cast<int>(filament_used[extr_nr]) << new_line;
            }
            if (mat_ids.size() == extruder_count && mat_ids[extr_nr] != "")
            {
                prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.GUID:" << mat_ids[extr_nr] << new_line;
            }
            prefix << ";EXTRUDER_TRAIN." << extr_nr << ".NOZZLE.DIAMETER:" << float(INT2MM(getNozzleSize(extr_nr))) << new_line;
        }
        prefix << ";BUILD_PLATE.INITIAL_TEMPERATURE:" << initial_bed_temp << new_line;

        if (print_time)
        {
            prefix << ";PRINT.TIME:" << static_cast<int>(*print_time) << new_line;
        }

        prefix << ";PRINT.SIZE.MIN.X:" << INT2MM(total_bounding_box.min.x) << new_line;
        prefix << ";PRINT.SIZE.MIN.Y:" << INT2MM(total_bounding_box.min.y) << new_line;
        prefix << ";PRINT.SIZE.MIN.Z:" << INT2MM(total_bounding_box.min.z) << new_line;
        prefix << ";PRINT.SIZE.MAX.X:" << INT2MM(total_bounding_box.max.x) << new_line;
        prefix << ";PRINT.SIZE.MAX.Y:" << INT2MM(total_bounding_box.max.y) << new_line;
        prefix << ";PRINT.SIZE.MAX.Z:" << INT2MM(total_bounding_box.max.z) << new_line;
        prefix << ";END_OF_HEADER" << new_line;
        break;
    default:
        prefix << ";TIME:" << ((print_time)? static_cast<int>(*print_time) : 6666) << new_line;
        if (flavor == EGCodeFlavor::ULTIGCODE)
        {
            prefix << ";MATERIAL:" << ((filament_used.size() >= 1)? static_cast<int>(filament_used[0]) : 6666) << new_line;
            prefix << ";MATERIAL2:" << ((filament_used.size() >= 2)? static_cast<int>(filament_used[1]) : 0) << new_line;

            prefix << ";NOZZLE_DIAMETER:" << float(INT2MM(getNozzleSize(0))) << new_line;
            // TODO: the second nozzle size isn't always initiated! ";NOZZLE_DIAMETER2:"
        }
        else if (flavor == EGCodeFlavor::REPRAP)
        {
            prefix << ";Filament used: " << ((filament_used.size() >= 1)? filament_used[0] / (1000 * extruder_attr[0].filament_area) : 0) << "m" << new_line;
            prefix << ";Layer height: " << layer_height << new_line;
        }
    }

    return prefix.str();
}


void GCodeExport::setLayerNr(unsigned int layer_nr_) {
    layer_nr = layer_nr_;
}

void GCodeExport::setOutputStream(std::ostream* stream)
{
    output_stream = stream;
    *output_stream << std::fixed;
}

bool GCodeExport::getExtruderIsUsed(const int extruder_nr) const
{
    assert(extruder_nr >= 0);
    assert(extruder_nr < MAX_EXTRUDERS);
    return extruder_attr[extruder_nr].is_used;
}

bool GCodeExport::getExtruderUsesTemp(const int extruder_nr) const
{
    return extruder_attr[extruder_nr].use_temp;
}

int GCodeExport::getNozzleSize(const int extruder_nr) const
{
    return extruder_attr[extruder_nr].nozzle_size;
}

Point GCodeExport::getExtruderOffset(const int id) const
{
    return extruder_attr[id].nozzle_offset;
}

std::string GCodeExport::getMaterialGUID(const int extruder_nr) const
{
    return extruder_attr[extruder_nr].material_guid;
}

Point GCodeExport::getGcodePos(const int64_t x, const int64_t y, const int extruder_train) const
{
    if (use_extruder_offset_to_offset_coords) { return Point(x,y) - getExtruderOffset(extruder_train); }
    else { return Point(x,y); }
}


void GCodeExport::setFlavor(EGCodeFlavor flavor)
{
    this->flavor = flavor;
    if (flavor == EGCodeFlavor::MACH3)
        for(int n=0; n<MAX_EXTRUDERS; n++)
            extruder_attr[n].extruderCharacter = 'A' + n;
    else
        for(int n=0; n<MAX_EXTRUDERS; n++)
            extruder_attr[n].extruderCharacter = 'E';
    if (flavor == EGCodeFlavor::ULTIGCODE || flavor == EGCodeFlavor::REPRAP_VOLUMATRIC)
    {
        is_volumatric = true;
    }
    else
    {
        is_volumatric = false;
    }

    if (flavor == EGCodeFlavor::BFB || flavor == EGCodeFlavor::REPRAP_VOLUMATRIC || flavor == EGCodeFlavor::ULTIGCODE)
    {
        firmware_retract = true;
    }
    else 
    {
        firmware_retract = false;
    }
}

EGCodeFlavor GCodeExport::getFlavor() const
{
    return this->flavor;
}

void GCodeExport::setZ(int z)
{
    this->current_layer_z = z;
}

Point3 GCodeExport::getPosition()
{
    return currentPosition;
}
Point GCodeExport::getPositionXY()
{
    return Point(currentPosition.x, currentPosition.y);
}

int GCodeExport::getPositionZ()
{
    return currentPosition.z;
}

int GCodeExport::getExtruderNr()
{
    return current_extruder;
}

void GCodeExport::setFilamentDiameter(unsigned int extruder, int diameter)
{
    double r = INT2MM(diameter) / 2.0;
    double area = M_PI * r * r;
    extruder_attr[extruder].filament_area = area;
}

double GCodeExport::getCurrentExtrudedVolume()
{
    double extrusion_amount = current_e_value;
    if (!firmware_retract)
    { // no E values are changed to perform a retraction
        extrusion_amount -= extruder_attr[current_extruder].retraction_e_amount_at_e_start; // subtract the increment in E which was used for the first unretraction instead of extrusion
        extrusion_amount += extruder_attr[current_extruder].retraction_e_amount_current; // add the decrement in E which the filament is behind on extrusion due to the last retraction
    }
    if (is_volumatric)
    {
        return extrusion_amount;
    }
    else
    {
        return extrusion_amount * extruder_attr[current_extruder].filament_area;
    }
}

double GCodeExport::eToMm(double e)
{
    if (is_volumatric)
    {
        return e / extruder_attr[current_extruder].filament_area;
    }
    else
    {
        return e;
    }
}

double GCodeExport::mm3ToE(double mm3)
{
    if (is_volumatric)
    {
        return mm3;
    }
    else
    {
        return mm3 / extruder_attr[current_extruder].filament_area;
    }
}

double GCodeExport::mmToE(double mm)
{
    if (is_volumatric)
    {
        return mm * extruder_attr[current_extruder].filament_area;
    }
    else
    {
        return mm;
    }
}


double GCodeExport::getTotalFilamentUsed(int extruder_nr)
{
    if (extruder_nr == current_extruder)
        return extruder_attr[extruder_nr].totalFilament + getCurrentExtrudedVolume();
    return extruder_attr[extruder_nr].totalFilament;
}

std::vector<double> GCodeExport::getTotalPrintTimePerFeature()
{
    return total_print_times;
}

double GCodeExport::getSumTotalPrintTimes()
{
    double sum = 0.0;
    for(double item : getTotalPrintTimePerFeature())
    {
        sum += item;
    }
    return sum;
}

void GCodeExport::resetTotalPrintTimeAndFilament()
{
    for(size_t i = 0; i < total_print_times.size(); i++)
    {
        total_print_times[i] = 0.0;
    }
    for(unsigned int e=0; e<MAX_EXTRUDERS; e++)
    {
        extruder_attr[e].totalFilament = 0.0;
        extruder_attr[e].currentTemperature = 0;
    }
    current_e_value = 0.0;
    estimateCalculator.reset();
}

void GCodeExport::updateTotalPrintTime()
{
    std::vector<double> estimates = estimateCalculator.calculate();
    for(size_t i = 0; i < estimates.size(); i++)
    {
        total_print_times[i] += estimates[i];
    }
    estimateCalculator.reset();
    writeTimeComment(getSumTotalPrintTimes());
}

void GCodeExport::writeComment(std::string comment)
{
    *output_stream << ";";
    for (unsigned int i = 0; i < comment.length(); i++)
    {
        if (comment[i] == '\n')
        {
            *output_stream << "\\n";
        }else{
            *output_stream << comment[i];
        }
    }
    *output_stream << new_line;
}

void GCodeExport::writeTimeComment(const double time)
{
    *output_stream << ";TIME_ELAPSED:" << time << new_line;
}

void GCodeExport::writeTypeComment(PrintFeatureType type)
{
    switch (type)
    {
        case PrintFeatureType::OuterWall:
            *output_stream << ";TYPE:WALL-OUTER" << new_line;
            break;
        case PrintFeatureType::InnerWall:
            *output_stream << ";TYPE:WALL-INNER" << new_line;
            break;
        case PrintFeatureType::Skin:
            *output_stream << ";TYPE:SKIN" << new_line;
            break;
        case PrintFeatureType::Support:
            *output_stream << ";TYPE:SUPPORT" << new_line;
            break;
        case PrintFeatureType::SkirtBrim:
            *output_stream << ";TYPE:SKIRT" << new_line;
            break;
        case PrintFeatureType::Infill:
            *output_stream << ";TYPE:FILL" << new_line;
            break;
        case PrintFeatureType::SupportInfill:
            *output_stream << ";TYPE:SUPPORT" << new_line;
            break;
        case PrintFeatureType::MoveCombing:
        case PrintFeatureType::MoveRetraction:
        default:
            // do nothing
            break;
    }
}


void GCodeExport::writeLayerComment(int layer_nr)
{
    *output_stream << ";LAYER:" << layer_nr << new_line;
}

void GCodeExport::writeLayerCountComment(int layer_count)
{
    *output_stream << ";LAYER_COUNT:" << layer_count << new_line;
}

void GCodeExport::writeLine(const char* line)
{
    *output_stream << line << new_line;
}

void GCodeExport::resetExtrusionValue()
{
    if (flavor != EGCodeFlavor::MAKERBOT && flavor != EGCodeFlavor::BFB)
    {
        *output_stream << "G92 " << extruder_attr[current_extruder].extruderCharacter << "0" << new_line;
        double current_extruded_volume = getCurrentExtrudedVolume();
        extruder_attr[current_extruder].totalFilament += current_extruded_volume;
        for (double& extruded_volume_at_retraction : extruder_attr[current_extruder].extruded_volume_at_previous_n_retractions)
        { // update the extruded_volume_at_previous_n_retractions only of the current extruder, since other extruders don't extrude the current volume
            extruded_volume_at_retraction -= current_extruded_volume;
        }
        current_e_value = 0.0;
        extruder_attr[current_extruder].retraction_e_amount_at_e_start = extruder_attr[current_extruder].retraction_e_amount_current;
    }
}

void GCodeExport::writeDelay(double timeAmount)
{
    *output_stream << "G4 P" << int(timeAmount * 1000) << new_line;
    estimateCalculator.addTime(timeAmount);
}

void GCodeExport::writeTravel(Point p, double speed)
{
    writeTravel(Point3(p.X, p.Y, current_layer_z), speed);
}
void GCodeExport::writeExtrusion(Point p, double speed, double extrusion_mm3_per_mm, PrintFeatureType feature)
{
    writeExtrusion(Point3(p.X, p.Y, current_layer_z), speed, extrusion_mm3_per_mm, feature);
}

void GCodeExport::writeTravel(Point3 p, double speed)
{
    if (flavor == EGCodeFlavor::BFB)
    {
        writeMoveBFB(p.x, p.y, p.z + isZHopped, speed, 0.0, PrintFeatureType::MoveCombing);
        return;
    }
    writeTravel(p.x, p.y, p.z + isZHopped, speed);
}

void GCodeExport::writeExtrusion(Point3 p, double speed, double extrusion_mm3_per_mm, PrintFeatureType feature)
{
    if (flavor == EGCodeFlavor::BFB)
    {
        writeMoveBFB(p.x, p.y, p.z, speed, extrusion_mm3_per_mm, feature);
        return;
    }
    writeExtrusion(p.x, p.y, p.z, speed, extrusion_mm3_per_mm, feature);
}

void GCodeExport::writeMoveBFB(int x, int y, int z, double speed, double extrusion_mm3_per_mm, PrintFeatureType feature)
{
    if (std::isinf(extrusion_mm3_per_mm))
    {
        logError("Extrusion rate is infinite!");
        assert(false && "Infinite extrusion move!");
        std::exit(1);
    }
    if (std::isnan(extrusion_mm3_per_mm))
    {
        logError("Extrusion rate is not a number!");
        assert(false && "NaN extrusion move!");
        std::exit(1);
    }

    double extrusion_per_mm = mm3ToE(extrusion_mm3_per_mm);

    Point gcode_pos = getGcodePos(x,y, current_extruder);

    //For Bits From Bytes machines, we need to handle this completely differently. As they do not use E values but RPM values.
    float fspeed = speed * 60;
    float rpm = extrusion_per_mm * speed * 60;
    const float mm_per_rpm = 4.0; //All BFB machines have 4mm per RPM extrusion.
    rpm /= mm_per_rpm;
    if (rpm > 0)
    {
        if (extruder_attr[current_extruder].retraction_e_amount_current)
        {
            if (currentSpeed != double(rpm))
            {
                //fprintf(f, "; %f e-per-mm %d mm-width %d mm/s\n", extrusion_per_mm, lineWidth, speed);
                //fprintf(f, "M108 S%0.1f\r\n", rpm);
                *output_stream << "M108 S" << PrecisionedDouble{1, rpm} << new_line;
                currentSpeed = double(rpm);
            }
            //Add M101 or M201 to enable the proper extruder.
            *output_stream << "M" << int((current_extruder + 1) * 100 + 1) << new_line;
            extruder_attr[current_extruder].retraction_e_amount_current = 0.0;
        }
        //Fix the speed by the actual RPM we are asking, because of rounding errors we cannot get all RPM values, but we have a lot more resolution in the feedrate value.
        // (Trick copied from KISSlicer, thanks Jonathan)
        fspeed *= (rpm / (roundf(rpm * 100) / 100));

        //Increase the extrusion amount to calculate the amount of filament used.
        Point3 diff = Point3(x,y,z) - getPosition();
        
        current_e_value += extrusion_per_mm * diff.vSizeMM();
    }
    else
    {
        //If we are not extruding, check if we still need to disable the extruder. This causes a retraction due to auto-retraction.
        if (!extruder_attr[current_extruder].retraction_e_amount_current)
        {
            *output_stream << "M103" << new_line;
            extruder_attr[current_extruder].retraction_e_amount_current = 1.0; // 1.0 used as stub; BFB doesn't use the actual retraction amount; it performs retraction on the firmware automatically
        }
    }
    *output_stream << "G1 X" << MMtoStream{gcode_pos.X} << " Y" << MMtoStream{gcode_pos.Y} << " Z" << MMtoStream{z};
    *output_stream << " F" << PrecisionedDouble{1, fspeed} << new_line;
    
    currentPosition = Point3(x, y, z);
    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), speed, feature);
}

void GCodeExport::writeTravel(int x, int y, int z, double speed)
{
    if (currentPosition.x == x && currentPosition.y == y && currentPosition.z == z)
        return;

#ifdef ASSERT_INSANE_OUTPUT
    assert(speed < 400 && speed > 1); // normal F values occurring in UM2 gcode (this code should not be compiled for release)
    assert(currentPosition != no_point3);
    assert(Point3(x, y, z) != no_point3);
    assert((Point3(x,y,z) - currentPosition).vSize() < MM2INT(300)); // no crazy positions (this code should not be compiled for release)
#endif //ASSERT_INSANE_OUTPUT

    const PrintFeatureType travel_move_type = extruder_attr[current_extruder].retraction_e_amount_current ? PrintFeatureType::MoveRetraction : PrintFeatureType::MoveCombing;
    const int display_width = extruder_attr[current_extruder].retraction_e_amount_current ? MM2INT(0.2) : MM2INT(0.1);
    CommandSocket::sendLineTo(travel_move_type, Point(x, y), display_width);

    *output_stream << "G0";
    writeFXYZE(speed, x, y, z, current_e_value, PrintFeatureType::MoveCombing);
}

void GCodeExport::writeExtrusion(int x, int y, int z, double speed, double extrusion_mm3_per_mm, PrintFeatureType feature)
{
    if (currentPosition.x == x && currentPosition.y == y && currentPosition.z == z)
        return;

#ifdef ASSERT_INSANE_OUTPUT
    assert(speed < 400 && speed > 1); // normal F values occurring in UM2 gcode (this code should not be compiled for release)
    assert(currentPosition != no_point3);
    assert(Point3(x, y, z) != no_point3);
    assert((Point3(x,y,z) - currentPosition).vSize() < MM2INT(300)); // no crazy positions (this code should not be compiled for release)
    assert(extrusion_mm3_per_mm >= 0.0);
#endif //ASSERT_INSANE_OUTPUT

    if (std::isinf(extrusion_mm3_per_mm))
    {
        logError("Extrusion rate is infinite!");
        assert(false && "Infinite extrusion move!");
        std::exit(1);
    }

    if (std::isnan(extrusion_mm3_per_mm))
    {
        logError("Extrusion rate is not a number!");
        assert(false && "NaN extrusion move!");
        std::exit(1);
    }

    if (extrusion_mm3_per_mm < 0.0)
    {
        logWarning("Warning! Negative extrusion move!\n");
    }

    double extrusion_per_mm = mm3ToE(extrusion_mm3_per_mm);

    if (isZHopped > 0)
    {
        writeZhopEnd();
    }

    Point3 diff = Point3(x,y,z) - currentPosition;

    writeUnretractionAndPrime();

    double new_e_value = current_e_value + extrusion_per_mm * diff.vSizeMM();

    *output_stream << "G1";
    writeFXYZE(speed, x, y, z, new_e_value, feature);
}

void GCodeExport::writeFXYZE(double speed, int x, int y, int z, double e, PrintFeatureType feature)
{
    if (currentSpeed != speed)
    {
        *output_stream << " F" << PrecisionedDouble{1, speed * 60};
        currentSpeed = speed;
    }

    Point gcode_pos = getGcodePos(x, y, current_extruder);
    total_bounding_box.include(Point3(gcode_pos.X, gcode_pos.Y, z));

    *output_stream << " X" << MMtoStream{gcode_pos.X} << " Y" << MMtoStream{gcode_pos.Y};
    if (z != currentPosition.z)
    {
        *output_stream << " Z" << MMtoStream{z};
    }
    if (e != current_e_value)
    {
        *output_stream << " " << extruder_attr[current_extruder].extruderCharacter << PrecisionedDouble{5, e};
    }
    *output_stream << new_line;
    
    currentPosition = Point3(x, y, z);
    current_e_value = e;
    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(x), INT2MM(y), INT2MM(z), eToMm(e)), speed, feature);
}

void GCodeExport::writeUnretractionAndPrime()
{
    const double prime_volume = extruder_attr[current_extruder].prime_volume;
    current_e_value += mm3ToE(prime_volume);
    if (extruder_attr[current_extruder].retraction_e_amount_current)
    {
        if (firmware_retract)
        { // note that BFB is handled differently
            *output_stream << "G11" << new_line;
            //Assume default UM2 retraction settings.
            if (prime_volume > 0)
            {
                *output_stream << "G1 F" << PrecisionedDouble{1, extruder_attr[current_extruder].last_retraction_prime_speed * 60} << " " << extruder_attr[current_extruder].extruderCharacter << PrecisionedDouble{5, current_e_value} << new_line;
                currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
            }
            estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), 25.0, PrintFeatureType::MoveRetraction);
        }
        else
        {
            current_e_value += extruder_attr[current_extruder].retraction_e_amount_current;
            *output_stream << "G1 F" << PrecisionedDouble{1, extruder_attr[current_extruder].last_retraction_prime_speed * 60} << " " << extruder_attr[current_extruder].extruderCharacter << PrecisionedDouble{5, current_e_value} << new_line;
            currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
            estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::MoveRetraction);
        }
        if (getCurrentExtrudedVolume() > 10000.0) //According to https://github.com/Ultimaker/CuraEngine/issues/14 having more then 21m of extrusion causes inaccuracies. So reset it every 10m, just to be sure.
        {
            resetExtrusionValue();
        }
        extruder_attr[current_extruder].retraction_e_amount_current = 0.0;
    }
    else if (prime_volume > 0.0)
    {
        *output_stream << "G1 F" << PrecisionedDouble{1, extruder_attr[current_extruder].last_retraction_prime_speed * 60} << " " << extruder_attr[current_extruder].extruderCharacter << PrecisionedDouble{5, current_e_value} << new_line;
        currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::NoneType);
    }
    extruder_attr[current_extruder].prime_volume = 0.0;
}

void GCodeExport::writeRetraction(const RetractionConfig& config, bool force, bool extruder_switch)
{
    ExtruderTrainAttributes& extr_attr = extruder_attr[current_extruder];

    if (flavor == EGCodeFlavor::BFB)//BitsFromBytes does automatic retraction.
    {
        if (extruder_switch)
        {
            if (!extr_attr.retraction_e_amount_current)
                *output_stream << "M103" << new_line;

            extr_attr.retraction_e_amount_current = 1.0; // 1.0 is a stub; BFB doesn't use the actual retracted amount; retraction is performed by firmware
        }
        return;
    }

    double old_retraction_e_amount = extr_attr.retraction_e_amount_current;
    double new_retraction_e_amount = mmToE(config.distance);
    double retraction_diff_e_amount = old_retraction_e_amount - new_retraction_e_amount;
    if (std::abs(retraction_diff_e_amount) < 0.000001)
    {
        return;
    }

    { // handle retraction limitation
        double current_extruded_volume = getCurrentExtrudedVolume();
        std::deque<double>& extruded_volume_at_previous_n_retractions = extr_attr.extruded_volume_at_previous_n_retractions;
        while (int(extruded_volume_at_previous_n_retractions.size()) > config.retraction_count_max && !extruded_volume_at_previous_n_retractions.empty()) 
        {
            // extruder switch could have introduced data which falls outside the retraction window
            // also the retraction_count_max could have changed between the last retraction and this
            extruded_volume_at_previous_n_retractions.pop_back();
        }
        if (!force && config.retraction_count_max <= 0)
        {
            return;
        }
        if (!force && int(extruded_volume_at_previous_n_retractions.size()) == config.retraction_count_max
            && current_extruded_volume < extruded_volume_at_previous_n_retractions.back() + config.retraction_extrusion_window * extr_attr.filament_area) 
        {
            return;
        }
        extruded_volume_at_previous_n_retractions.push_front(current_extruded_volume);
        if (int(extruded_volume_at_previous_n_retractions.size()) == config.retraction_count_max + 1) 
        {
            extruded_volume_at_previous_n_retractions.pop_back();
        }
    }

    if (firmware_retract)
    {
        if (extruder_switch && extr_attr.retraction_e_amount_current) 
        {
            return; 
        }
        *output_stream << "G10";
        if (extruder_switch)
        {
            *output_stream << " S1";
        }
        *output_stream << new_line;
        //Assume default UM2 retraction settings.
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value + retraction_diff_e_amount)), 25, PrintFeatureType::MoveRetraction); // TODO: hardcoded values!
    }
    else
    {
        double speed = ((retraction_diff_e_amount < 0.0)? config.speed : extr_attr.last_retraction_prime_speed) * 60;
        current_e_value += retraction_diff_e_amount;
        *output_stream << "G1 F" << PrecisionedDouble{1, speed} << " "
            << extr_attr.extruderCharacter << PrecisionedDouble{5, current_e_value} << new_line;
        currentSpeed = speed;
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::MoveRetraction);
        extr_attr.last_retraction_prime_speed = config.primeSpeed;
    }

    extr_attr.retraction_e_amount_current = new_retraction_e_amount; // suppose that for UM2 the retraction amount in the firmware is equal to the provided amount
    extr_attr.prime_volume += config.prime_volume;

}

void GCodeExport::writeZhopStart(int hop_height)
{
    if (hop_height > 0)
    {
        isZHopped = hop_height;
        *output_stream << "G1 Z" << MMtoStream{current_layer_z + isZHopped} << new_line;
        total_bounding_box.includeZ(current_layer_z + isZHopped);
    }
}

void GCodeExport::writeZhopEnd()
{
    if (isZHopped)
    {
        isZHopped = 0;
        currentPosition.z = current_layer_z;
        *output_stream << "G1 Z" << MMtoStream{current_layer_z} << new_line;
    }
}

void GCodeExport::startExtruder(int new_extruder)
{
    if (new_extruder != current_extruder) // wouldn't be the case on the very first extruder start if it's extruder 0
    {
        extruder_attr[new_extruder].is_used = true;
        if (flavor == EGCodeFlavor::MAKERBOT)
        {
            *output_stream << "M135 T" << new_extruder << new_line;
        }
        else
        {
            *output_stream << "T" << new_extruder << new_line;
        }
    }

    current_extruder = new_extruder;

    assert(getCurrentExtrudedVolume() == 0.0 && "Just after an extruder switch we haven't extruded anything yet!");
    resetExtrusionValue(); // zero the E value on the new extruder, just to be sure

    writeCode(extruder_attr[new_extruder].start_code.c_str());
    CommandSocket::setExtruderForSend(new_extruder);
    CommandSocket::setSendCurrentPosition( getPositionXY() );

    //Change the Z position so it gets re-writting again. We do not know if the switch code modified the Z position.
    currentPosition.z += 1;
}

void GCodeExport::switchExtruder(int new_extruder, const RetractionConfig& retraction_config_old_extruder)
{
    if (current_extruder == new_extruder)
        return;

    bool force = true;
    bool extruder_switch = true;
    writeRetraction(retraction_config_old_extruder, force, extruder_switch);

    resetExtrusionValue(); // zero the E value on the old extruder, so that the current_e_value is registered on the old extruder

    int old_extruder = current_extruder;

    writeCode(extruder_attr[old_extruder].end_code.c_str());

    startExtruder(new_extruder);
}

void GCodeExport::writeCode(const char* str)
{
    *output_stream << str << new_line;
}

void GCodeExport::writePrimeTrain(double travel_speed)
{
    if (extruder_attr[current_extruder].is_primed)
    { // extruder is already primed once!
        return;
    }
    if (extruder_attr[current_extruder].is_prime_blob_enabled)
    { // only move to prime position if we do a blob/poop
        // ideally the prime position would be respected whether we do a blob or not,
        // but the frontend currently doesn't support a value function of an extruder setting depending on an fdmprinter setting,
        // which is needed to automatically ignore the prime position for the UM3 machine when blob is disabled
        Point3 prime_pos = extruder_attr[current_extruder].prime_pos;
        if (!extruder_attr[current_extruder].prime_pos_is_abs)
        {
            prime_pos += currentPosition;
        }
        writeTravel(prime_pos, travel_speed);
    }

    if (flavor == EGCodeFlavor::GRIFFIN)
    {
        std::string command = "G280";
        if (!extruder_attr[current_extruder].is_prime_blob_enabled)
        {
            command += " S1";  // use S1 to disable prime blob
        }
        *output_stream << command << new_line;
    }
    else
    {
        // there is no prime gcode for other firmware versions...
    }

    extruder_attr[current_extruder].is_primed = true;
}


void GCodeExport::writeFanCommand(double speed)
{
    if (currentFanSpeed == speed)
        return;
    if (speed > 0)
    {
        if (flavor == EGCodeFlavor::MAKERBOT)
            *output_stream << "M126 T0" << new_line; //value = speed * 255 / 100 // Makerbot cannot set fan speed...;
        else
            *output_stream << "M106 S" << PrecisionedDouble{1, speed * 255 / 100} << new_line;
    }
    else
    {
        if (flavor == EGCodeFlavor::MAKERBOT)
            *output_stream << "M127 T0" << new_line;
        else
            *output_stream << "M107" << new_line;
    }
    currentFanSpeed = speed;
}

void GCodeExport::writeTemperatureCommand(int extruder, double temperature, bool wait)
{
    if (!extruder_attr[extruder].use_temp)
    {
        return;
    }

    if (!wait && extruder_attr[extruder].currentTemperature == temperature)
    {
        return;
    }

    if (wait)
        *output_stream << "M109";
    else
        *output_stream << "M104";
    if (extruder != current_extruder)
        *output_stream << " T" << extruder;
#ifdef ASSERT_INSANE_OUTPUT
    assert(temperature >= 0);
#endif // ASSERT_INSANE_OUTPUT
    *output_stream << " S" << PrecisionedDouble{1, temperature} << new_line;
    extruder_attr[extruder].currentTemperature = temperature;
}

void GCodeExport::writeBedTemperatureCommand(double temperature, bool wait)
{
    if (flavor == EGCodeFlavor::ULTIGCODE)
    { // The UM2 family doesn't support temperature commands (they are fixed in the firmware)
        return;
    }

    if (wait)
        *output_stream << "M190 S";
    else
        *output_stream << "M140 S";
    *output_stream << PrecisionedDouble{1, temperature} << new_line;
}

void GCodeExport::writeAcceleration(double acceleration, bool for_travel_moves)
{
    if (getFlavor() == EGCodeFlavor::REPETIER)
    {
        int m_code = 0;
        if (for_travel_moves)
        {
            if (current_travel_acceleration != acceleration)
            {
                m_code = 202;   // set travel acceleration
                current_travel_acceleration = acceleration;
            }
        }
        else
        {
            if (current_acceleration != acceleration)
            {
                m_code = 201;  // set print acceleration
                current_acceleration = acceleration;
            }
        }
        if (m_code != 0)
        {
            *output_stream << "M" << m_code << " X" << PrecisionedDouble{0, acceleration} << " Y" << PrecisionedDouble{0, acceleration} << new_line;
            estimateCalculator.setAcceleration(acceleration);
        }
    }
    else
    {
        if (current_acceleration != acceleration)
        {
            *output_stream << "M204 S" << PrecisionedDouble{0, acceleration} << new_line; // Print and Travel acceleration
            current_acceleration = acceleration;
            estimateCalculator.setAcceleration(acceleration);
        }
    }
}

void GCodeExport::writeJerk(double jerk)
{
    if (current_jerk != jerk)
    {
        if (getFlavor() == EGCodeFlavor::REPETIER)
        {
            *output_stream << "M207 X";
        }
        else
        {
            *output_stream << "M205 X";
        }
        *output_stream << PrecisionedDouble{2, jerk} << new_line;
        current_jerk = jerk;
        estimateCalculator.setMaxXyJerk(jerk);
    }
}

void GCodeExport::writeMaxZFeedrate(double max_z_feedrate)
{
    if (current_max_z_feedrate != max_z_feedrate)
    {
        *output_stream << "M203 Z" << PrecisionedDouble{2, max_z_feedrate} << new_line;
        current_max_z_feedrate = max_z_feedrate;
        estimateCalculator.setMaxZFeedrate(max_z_feedrate);
    }
}

double GCodeExport::getCurrentMaxZFeedrate()
{
    return current_max_z_feedrate;
}

void GCodeExport::finalize(const char* endCode)
{
    writeFanCommand(0);
    writeCode(endCode);
    int64_t print_time = getSumTotalPrintTimes();
    int mat_0 = getTotalFilamentUsed(0);
    log("Print time: %d\n", print_time);
    log("Print time (readable): %dh %dm %ds\n", print_time / 60 / 60, (print_time / 60) % 60, print_time % 60);
    log("Filament: %d\n", mat_0);
    for(int n=1; n<MAX_EXTRUDERS; n++)
        if (getTotalFilamentUsed(n) > 0)
            log("Filament%d: %d\n", n + 1, int(getTotalFilamentUsed(n)));
    output_stream->flush();
}

}//namespace cura

