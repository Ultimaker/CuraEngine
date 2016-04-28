/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdarg.h>
#include <iomanip>
#include <cmath>

#include "gcodeExport.h"
#include "utils/logoutput.h"
#include "PrintFeature.h"
#include "utils/Date.h"

namespace cura {

GCodeExport::GCodeExport()
: output_stream(&std::cout)
, currentPosition(0,0,MM2INT(20))
, layer_nr(0)
{
    current_e_value = 0;
    current_extruder = 0;
    currentFanSpeed = -1;

    totalPrintTime = 0.0;

    currentSpeed = 1;
    isZHopped = 0;
    setFlavor(EGCodeFlavor::REPRAP);
    initial_bed_temp = 0;

    extruder_count = 0;
}

GCodeExport::~GCodeExport()
{
}

void GCodeExport::preSetup(MeshGroup* settings)
{
    setFlavor(settings->getSettingAsGCodeFlavor("machine_gcode_flavor"));
    use_extruder_offset_to_offset_coords = settings->getSettingBoolean("machine_use_extruder_offset_to_offset_coords");

    extruder_count = settings->getSettingAsCount("machine_extruder_count");

    for (unsigned int n = 0; n < extruder_count; n++)
    {
        ExtruderTrain* train = settings->getExtruderTrain(n);
        setFilamentDiameter(n, train->getSettingInMicrons("material_diameter")); 

        extruder_attr[n].nozzle_size = train->getSettingInMicrons("machine_nozzle_size");
        extruder_attr[n].nozzle_offset = Point(train->getSettingInMicrons("machine_nozzle_offset_x"), train->getSettingInMicrons("machine_nozzle_offset_y"));

        extruder_attr[n].start_code = train->getSettingString("machine_extruder_start_code");
        extruder_attr[n].end_code = train->getSettingString("machine_extruder_end_code");

        extruder_attr[n].extruder_switch_retraction_config.distance = train->getSettingInMillimeters("switch_extruder_retraction_amount"); 
        extruder_attr[n].extruder_switch_retraction_config.prime_volume = 0.0;
        extruder_attr[n].extruder_switch_retraction_config.speed = train->getSettingInMillimetersPerSecond("switch_extruder_retraction_speed");
        extruder_attr[n].extruder_switch_retraction_config.primeSpeed = train->getSettingInMillimetersPerSecond("switch_extruder_prime_speed");
        extruder_attr[n].extruder_switch_retraction_config.zHop = 1000; // TODO: make configurable
        extruder_attr[n].extruder_switch_retraction_config.retraction_count_max = 9999999; // extruder switch retraction is never limited
        extruder_attr[n].extruder_switch_retraction_config.retraction_extrusion_window = 99999.9; // so that extruder switch retractions won't affect the retraction buffer (extruded_volume_at_previous_n_retractions)
        extruder_attr[n].extruder_switch_retraction_config.retraction_min_travel_distance = 0; // no limitation on travel distance for an extruder switch retract

        extruder_attr[n].last_retraction_prime_speed = train->getSettingInMillimetersPerSecond("retraction_prime_speed"); // the alternative would be switch_extruder_prime_speed, but dual extrusion might not even be configured...
    }
    machine_dimensions.x = settings->getSettingInMicrons("machine_width");
    machine_dimensions.y = settings->getSettingInMicrons("machine_depth");
    machine_dimensions.z = settings->getSettingInMicrons("machine_height");

    machine_name = settings->getSettingString("machine_name");

    if (flavor == EGCodeFlavor::BFB)
    {
        new_line = "\r\n";
    }
    else 
    {
        new_line = "\n";
    }
}

void GCodeExport::setInitialTemps(const MeshGroup& settings)
{
    for (unsigned int extr_nr = 0; extr_nr < extruder_count; extr_nr++)
    {
        const ExtruderTrain* extr_train = settings.getExtruderTrain(extr_nr);
        assert(extr_train);
        double temp = extr_train->getSettingInDegreeCelsius((extr_nr == 0)? "material_print_temperature" : "material_standby_temperature");
        setInitialTemp(extr_nr, temp);
    }

    initial_bed_temp = settings.getSettingInDegreeCelsius("material_bed_temperature");
}

void GCodeExport::setInitialTemp(int extruder_nr, double temp)
{
    extruder_attr[extruder_nr].initial_temp = temp;
    if (flavor == EGCodeFlavor::GRIFFIN || flavor == EGCodeFlavor::ULTIGCODE)
    {
        extruder_attr[extruder_nr].currentTemperature = temp;
    }
}


std::string GCodeExport::getFileHeader(const double* print_time, const std::vector<double>& filament_used, const std::vector<int16_t>& mat_ids)
{
    std::ostringstream prefix;
    switch (flavor)
    {
    case EGCodeFlavor::GRIFFIN:
        prefix << ";START_OF_HEADER" << new_line;
        prefix << ";HEADER_VERSION:0.1" << new_line;
        prefix << ";FLAVOR:" << toString(flavor) << new_line;
        prefix << ";GENERATOR.NAME:Cura_SteamEngine" << new_line;
        prefix << ";GENERATOR.VERSION:" << VERSION << new_line;
        prefix << ";GENERATOR.BUILD_DATE:" << Date::getDate().toStringDashed() << new_line;
        prefix << ";TARGET_MACHINE.NAME:" << machine_name << new_line;

        for (unsigned int extr_nr = 0; extr_nr < extruder_count; extr_nr++)
        {
            prefix << ";EXTRUDER_TRAIN." << extr_nr << ".INITIAL_TEMPERATURE:" << extruder_attr[extr_nr].initial_temp << new_line;
            if (filament_used.size() == extruder_count)
            {
                prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.VOLUME_USED:" << static_cast<int>(filament_used[extr_nr]) << new_line;
            }
            if (mat_ids.size() == extruder_count)
            {
                prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.GUID:" << mat_ids[extr_nr] << new_line; // TODO: convert to hexadecimal format
            }
            prefix << ";EXTRUDER_TRAIN." << extr_nr << ".NOZZLE.DIAMETER:" << float(INT2MM(getNozzleSize(extr_nr))) << new_line;
        }
        prefix << ";BUILD_PLATE.INITIAL_TEMPERATURE:" << initial_bed_temp << new_line;

        if (print_time)
        {
            prefix << ";PRINT.TIME:" << static_cast<int>(*print_time) << new_line;
        }

        prefix << ";PRINT.SIZE.MIN.X:0" << new_line;
        prefix << ";PRINT.SIZE.MIN.Y:0" << new_line;
        prefix << ";PRINT.SIZE.MIN.Z:0" << new_line;
        prefix << ";PRINT.SIZE.MAX.X:" << machine_dimensions.x << new_line;
        prefix << ";PRINT.SIZE.MAX.Y:" << machine_dimensions.y << new_line;
        prefix << ";PRINT.SIZE.MAX.Z:" << machine_dimensions.z << new_line;
        prefix << ";END_OF_HEADER" << new_line;
        return prefix.str();
    default:
        prefix << ";FLAVOR:" << toString(flavor) << new_line;
        prefix << ";TIME:" << ((print_time)? static_cast<int>(*print_time) : 6666) << new_line;
        if (flavor == EGCodeFlavor::ULTIGCODE)
        {
            prefix << ";MATERIAL:" << ((filament_used.size() >= 1)? static_cast<int>(filament_used[0]) : 6666) << new_line;
            prefix << ";MATERIAL2:" << ((filament_used.size() >= 2)? static_cast<int>(filament_used[1]) : 0) << new_line;

            prefix << ";NOZZLE_DIAMETER:" << float(INT2MM(getNozzleSize(0))) << new_line;
            // TODO: the second nozzle size isn't always initiated! ";NOZZLE_DIAMETER2:"
        }
        return prefix.str();
    }
}


void GCodeExport::setLayerNr(unsigned int layer_nr_) {
    layer_nr = layer_nr_;
}

void GCodeExport::setOutputStream(std::ostream* stream)
{
    output_stream = stream;
    *output_stream << std::fixed;
}

int GCodeExport::getNozzleSize(int extruder_idx)
{
    return extruder_attr[extruder_idx].nozzle_size;
}

Point GCodeExport::getExtruderOffset(int id)
{
    return extruder_attr[id].nozzle_offset;
}

Point GCodeExport::getGcodePos(int64_t x, int64_t y, int extruder_train)
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

EGCodeFlavor GCodeExport::getFlavor()
{
    return this->flavor;
}

void GCodeExport::setZ(int z)
{
    this->zPos = z;
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

double GCodeExport::getTotalPrintTime()
{
    return totalPrintTime;
}

void GCodeExport::resetTotalPrintTimeAndFilament()
{
    totalPrintTime = 0;
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
    totalPrintTime += estimateCalculator.calculate();
    estimateCalculator.reset();
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

void GCodeExport::writeTypeComment(const char* type)
{
    *output_stream << ";TYPE:" << type << new_line;
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
        case PrintFeatureType::Skirt:
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

void GCodeExport::writeMove(Point p, double speed, double extrusion_mm3_per_mm)
{
    writeMove(p.X, p.Y, zPos, speed, extrusion_mm3_per_mm);
}

void GCodeExport::writeMove(Point3 p, double speed, double extrusion_mm3_per_mm)
{
    writeMove(p.x, p.y, p.z, speed, extrusion_mm3_per_mm);
}

void GCodeExport::writeMoveBFB(int x, int y, int z, double speed, double extrusion_mm3_per_mm)
{
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
                *output_stream << "M108 S" << std::setprecision(1) << rpm << new_line;
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
    *output_stream << std::setprecision(3) << 
        "G1 X" << INT2MM(gcode_pos.X) << 
        " Y" << INT2MM(gcode_pos.Y) << 
        " Z" << INT2MM(z) << std::setprecision(1) << " F" << fspeed << new_line;
    
    currentPosition = Point3(x, y, z);
    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), speed);
}

void GCodeExport::writeMove(int x, int y, int z, double speed, double extrusion_mm3_per_mm)
{
    if (currentPosition.x == x && currentPosition.y == y && currentPosition.z == z)
        return;

#ifdef ASSERT_INSANE_OUTPUT
    assert(speed < 200 && speed > 1); // normal F values occurring in UM2 gcode (this code should not be compiled for release)
    assert(currentPosition != no_point3);
    assert((Point3(x,y,z) - currentPosition).vSize() < MM2INT(300)); // no crazy positions (this code should not be compiled for release)
#endif //ASSERT_INSANE_OUTPUT

    if (extrusion_mm3_per_mm < 0)
        logWarning("Warning! Negative extrusion move!");

    if (flavor == EGCodeFlavor::BFB)
    {
        writeMoveBFB(x, y, z, speed, extrusion_mm3_per_mm);
        return;
    }

    double extrusion_per_mm = mm3ToE(extrusion_mm3_per_mm);

    Point gcode_pos = getGcodePos(x,y, current_extruder);

    if (extrusion_mm3_per_mm > 0.000001)
    {
        Point3 diff = Point3(x,y,z) - getPosition();
        if (isZHopped > 0)
        {
            *output_stream << std::setprecision(3) << "G1 Z" << INT2MM(currentPosition.z) << new_line;
            isZHopped = 0;
        }
        double prime_volume = extruder_attr[current_extruder].prime_volume;
        current_e_value += mm3ToE(prime_volume);
        if (extruder_attr[current_extruder].retraction_e_amount_current)
        {
            if (firmware_retract)
            { // note that BFB is handled differently
                *output_stream << "G11" << new_line;
                //Assume default UM2 retraction settings.
                if (prime_volume > 0)
                {
                    *output_stream << "G1 F" << (extruder_attr[current_extruder].last_retraction_prime_speed * 60) << " " << extruder_attr[current_extruder].extruderCharacter << std::setprecision(5) << current_e_value << new_line;
                    currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
                }
                estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), 25.0);
            }
            else
            {
                current_e_value += extruder_attr[current_extruder].retraction_e_amount_current;
                *output_stream << "G1 F" << (extruder_attr[current_extruder].last_retraction_prime_speed * 60) << " " << extruder_attr[current_extruder].extruderCharacter << std::setprecision(5) << current_e_value << new_line;
                currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
                estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed);
            }
            if (getCurrentExtrudedVolume() > 10000.0) //According to https://github.com/Ultimaker/CuraEngine/issues/14 having more then 21m of extrusion causes inaccuracies. So reset it every 10m, just to be sure.
            {
                resetExtrusionValue();
            }
            extruder_attr[current_extruder].retraction_e_amount_current = 0.0;
        }
        else if (prime_volume > 0.0)
        {
            *output_stream << "G1 F" << (extruder_attr[current_extruder].last_retraction_prime_speed * 60) << " " << extruder_attr[current_extruder].extruderCharacter << std::setprecision(5) << current_e_value << new_line;
            currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
            estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed);
        }
        extruder_attr[current_extruder].prime_volume = 0.0;
        current_e_value += extrusion_per_mm * diff.vSizeMM();
        *output_stream << "G1";
    }
    else
    {
        *output_stream << "G0";

        if (CommandSocket::isInstantiated()) 
        {
            // we should send this travel as a non-retraction move
            cura::Polygons travelPoly;
            PolygonRef travel = travelPoly.newPoly();
            travel.add(Point(currentPosition.x, currentPosition.y));
            travel.add(Point(x, y));
            CommandSocket::getInstance()->sendPolygons(extruder_attr[current_extruder].retraction_e_amount_current ? PrintFeatureType::MoveRetraction : PrintFeatureType::MoveCombing, layer_nr, travelPoly, extruder_attr[current_extruder].retraction_e_amount_current ? MM2INT(0.2) : MM2INT(0.1));
        }
    }

    if (currentSpeed != speed)
    {
        *output_stream << " F" << (speed * 60);
        currentSpeed = speed;
    }

    *output_stream << std::setprecision(3) << 
        " X" << INT2MM(gcode_pos.X) << 
        " Y" << INT2MM(gcode_pos.Y);
    if (z != currentPosition.z + isZHopped)
        *output_stream << " Z" << INT2MM(z + isZHopped);
    if (extrusion_mm3_per_mm > 0.000001)
        *output_stream << " " << extruder_attr[current_extruder].extruderCharacter << std::setprecision(5) << current_e_value;
    *output_stream << new_line;
    
    currentPosition = Point3(x, y, z);
    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), speed);
}

void GCodeExport::writeRetraction(RetractionConfig* config, bool force, bool extruder_switch)
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
    double new_retraction_e_amount = mmToE(config->distance);
    double retraction_diff_e_amount = old_retraction_e_amount - new_retraction_e_amount;
    if (std::abs(retraction_diff_e_amount) < 0.000001)
    {
        return;
    }

    { // handle retraction limitation
        double current_extruded_volume = getCurrentExtrudedVolume();
        std::deque<double>& extruded_volume_at_previous_n_retractions = extr_attr.extruded_volume_at_previous_n_retractions;
        while (int(extruded_volume_at_previous_n_retractions.size()) > config->retraction_count_max && !extruded_volume_at_previous_n_retractions.empty()) 
        {
            // extruder switch could have introduced data which falls outside the retraction window
            // also the retraction_count_max could have changed between the last retraction and this
            extruded_volume_at_previous_n_retractions.pop_back();
        }
        if (!force && config->retraction_count_max <= 0)
        {
            return;
        }
        if (!force && int(extruded_volume_at_previous_n_retractions.size()) == config->retraction_count_max
            && current_extruded_volume < extruded_volume_at_previous_n_retractions.back() + config->retraction_extrusion_window * extr_attr.filament_area) 
        {
            return;
        }
        extruded_volume_at_previous_n_retractions.push_front(current_extruded_volume);
        if (int(extruded_volume_at_previous_n_retractions.size()) == config->retraction_count_max + 1) 
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
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value + retraction_diff_e_amount)), 25); // TODO: hardcoded values!
    }
    else
    {
        double speed = ((retraction_diff_e_amount < 0.0)? config->speed : extr_attr.last_retraction_prime_speed) * 60;
        current_e_value += retraction_diff_e_amount;
        *output_stream << "G1 F" << speed << " "
            << extr_attr.extruderCharacter << std::setprecision(5) << current_e_value << new_line;
        currentSpeed = speed;
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed);
        extr_attr.last_retraction_prime_speed = config->primeSpeed;
    }

    extr_attr.retraction_e_amount_current = new_retraction_e_amount; // suppose that for UM2 the retraction amount in the firmware is equal to the provided amount
    extr_attr.prime_volume += config->prime_volume;

    if (config->zHop > 0)
    {
        isZHopped = config->zHop;
        *output_stream << std::setprecision(3) << "G1 Z" << INT2MM(currentPosition.z + isZHopped) << new_line;
    }
}

void GCodeExport::writeRetraction_extruderSwitch()
{
    ExtruderTrainAttributes& extr_attr = extruder_attr[current_extruder];
    RetractionConfig* config = &extr_attr.extruder_switch_retraction_config;

    writeRetraction(config, true, true);
}

void GCodeExport::switchExtruder(int new_extruder)
{
    if (current_extruder == new_extruder)
        return;

    writeRetraction_extruderSwitch();

    int old_extruder = current_extruder;
    current_extruder = new_extruder;


    writeCode(extruder_attr[old_extruder].end_code.c_str());
    if (flavor == EGCodeFlavor::MAKERBOT)
    {
        *output_stream << "M135 T" << current_extruder << new_line;
    }
    else
    {
        *output_stream << "T" << current_extruder << new_line;
    }

    resetExtrusionValue(); // zero the E value on the new extruder
    writeCode(extruder_attr[new_extruder].start_code.c_str());

    //Change the Z position so it gets re-writting again. We do not know if the switch code modified the Z position.
    currentPosition.z += 1;
}

void GCodeExport::writeCode(const char* str)
{
    *output_stream << str << new_line;
}

void GCodeExport::writePrimeTrain()
{
    *output_stream << "G280" << new_line;
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
            *output_stream << "M106 S" << (speed * 255 / 100) << new_line;
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
    if (!wait && extruder_attr[extruder].currentTemperature == temperature)
        return;
    
    if (wait)
        *output_stream << "M109";
    else
        *output_stream << "M104";
    if (extruder != current_extruder)
        *output_stream << " T" << extruder;
    *output_stream << " S" << temperature << new_line;
    extruder_attr[extruder].currentTemperature = temperature;
}

void GCodeExport::writeBedTemperatureCommand(double temperature, bool wait)
{
    if (wait)
        *output_stream << "M190 S";
    else
        *output_stream << "M140 S";
    *output_stream << temperature << new_line;
}

void GCodeExport::finalize(double moveSpeed, const char* endCode)
{
    writeFanCommand(0);
    writeCode(endCode);
    long print_time = getTotalPrintTime();
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

