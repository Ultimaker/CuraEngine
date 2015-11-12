/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdarg.h>
#include <iomanip>
#include <cmath>

#include "gcodeExport.h"
#include "utils/logoutput.h"

namespace cura {

GCodeExport::GCodeExport()
: output_stream(&std::cout)
, currentPosition(0,0,MM2INT(20))
, commandSocket(nullptr)
, layer_nr(0)
{
    current_e_value = 0;
    current_extruder = 0;
    currentFanSpeed = -1;
    
    totalPrintTime = 0.0;
    
    currentSpeed = 1;
    retractionPrimeSpeed = 1;
    isZHopped = 0;
    setFlavor(EGCodeFlavor::REPRAP);
}

GCodeExport::~GCodeExport()
{
}

void GCodeExport::setCommandSocketAndLayerNr(CommandSocket* commandSocket_, unsigned int layer_nr_) {
    commandSocket = commandSocket_;
    layer_nr = layer_nr_;
}

void GCodeExport::setOutputStream(std::ostream* stream)
{
    output_stream = stream;
    *output_stream << std::fixed;
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

double GCodeExport::getCurrentExtrudedVolume(unsigned int extruder)
{
    double extrusion_amount = current_e_value + extruder_attr[current_extruder].isRetracted;
    if (is_volumatric)
    {
        return extrusion_amount;
    }
    else
    {
        return extrusion_amount * extruder_attr[extruder].filament_area;
    }
}


double GCodeExport::getTotalFilamentUsed(int e)
{
    if (e == current_extruder)
        return extruder_attr[e].totalFilament + getCurrentExtrudedVolume(e);
    return extruder_attr[e].totalFilament;
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
    *output_stream << "\n";
}

void GCodeExport::writeTypeComment(const char* type)
{
    *output_stream << ";TYPE:" << type << "\n";
}
void GCodeExport::writeLayerComment(int layer_nr)
{
    *output_stream << ";LAYER:" << layer_nr << "\n";
}

void GCodeExport::writeLayerCountComment(int layer_count)
{
    *output_stream << ";LAYER_COUNT:" << layer_count << "\n";
}

void GCodeExport::writeLine(const char* line)
{
    *output_stream << line << "\n";
}

void GCodeExport::resetExtrusionValue()
{
    if (current_e_value != 0.0 && flavor != EGCodeFlavor::MAKERBOT && flavor != EGCodeFlavor::BFB)
    {
        *output_stream << "G92 " << extruder_attr[current_extruder].extruderCharacter << "0\n";
        double current_extruded_volume = getCurrentExtrudedVolume(current_extruder);
        extruder_attr[current_extruder].totalFilament += current_extruded_volume;
        for (double& extruded_volume_at_retraction : extruder_attr[current_extruder].extruded_volume_at_previous_n_retractions)
        { // update the extruded_volume_at_previous_n_retractions only of the current extruder, since other extruders don't extrude the current volume
            extruded_volume_at_retraction -= current_extruded_volume;
        }
        current_e_value = 0.0;
    }
}

void GCodeExport::writeDelay(double timeAmount)
{
    *output_stream << "G4 P" << int(timeAmount * 1000) << "\n";
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
    double extrusion_per_mm = extrusion_mm3_per_mm;
    if (!is_volumatric)
    {
        extrusion_per_mm = extrusion_mm3_per_mm / extruder_attr[current_extruder].filament_area;
    }
    
    Point gcode_pos = getGcodePos(x,y, current_extruder);
    
    //For Bits From Bytes machines, we need to handle this completely differently. As they do not use E values but RPM values.
    float fspeed = speed * 60;
    float rpm = extrusion_per_mm * speed * 60;
    const float mm_per_rpm = 4.0; //All BFB machines have 4mm per RPM extrusion.
    rpm /= mm_per_rpm;
    if (rpm > 0)
    {
        if (extruder_attr[current_extruder].isRetracted)
        {
            if (currentSpeed != double(rpm))
            {
                //fprintf(f, "; %f e-per-mm %d mm-width %d mm/s\n", extrusion_per_mm, lineWidth, speed);
                //fprintf(f, "M108 S%0.1f\r\n", rpm);
                *output_stream << "M108 S" << std::setprecision(1) << rpm << "\r\n";
                currentSpeed = double(rpm);
            }
            //Add M101 or M201 to enable the proper extruder.
            *output_stream << "M" << int((current_extruder + 1) * 100 + 1) << "\r\n";
            extruder_attr[current_extruder].isRetracted = 0.0;
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
        if (!extruder_attr[current_extruder].isRetracted)
        {
            *output_stream << "M103\r\n";
            extruder_attr[current_extruder].isRetracted = 1.0; // 1.0 used as stub; BFB doesn't use the actual retraction amount; it performs retraction on the firmware automatically
        }
    }
    *output_stream << std::setprecision(3) << 
        "G1 X" << INT2MM(gcode_pos.X) << 
        " Y" << INT2MM(gcode_pos.Y) << 
        " Z" << INT2MM(z) << std::setprecision(1) << " F" << fspeed << "\r\n";
    
    currentPosition = Point3(x, y, z);
    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), current_e_value), speed);
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

    double extrusion_per_mm = extrusion_mm3_per_mm;
    if (!is_volumatric)
    {
        extrusion_per_mm = extrusion_mm3_per_mm / extruder_attr[current_extruder].filament_area;
    }

    Point gcode_pos = getGcodePos(x,y, current_extruder);

    if (extrusion_mm3_per_mm > 0.000001)
    {
        Point3 diff = Point3(x,y,z) - getPosition();
        if (isZHopped > 0)
        {
            if (std::isinf(currentPosition.z))
            {
                logError("Error! No Z position set yet!");
                assert(!std::isinf(currentPosition.z));
            }
            *output_stream << std::setprecision(3) << "G1 Z" << INT2MM(currentPosition.z) << "\n";
            isZHopped = 0;
        }
        double prime_amount = extruder_attr[current_extruder].prime_amount;
        current_e_value += (is_volumatric) ? prime_amount : prime_amount / extruder_attr[current_extruder].filament_area;   
        if (extruder_attr[current_extruder].isRetracted)
        {
            if (flavor == EGCodeFlavor::ULTIGCODE || flavor == EGCodeFlavor::REPRAP_VOLUMATRIC)
            {
                *output_stream << "G11\n";
                //Assume default UM2 retraction settings.
                if (prime_amount > 0)
                {
                    *output_stream << "G1 F" << (retractionPrimeSpeed * 60) << " " << extruder_attr[current_extruder].extruderCharacter << std::setprecision(5) << current_e_value << "\n";
                    currentSpeed = retractionPrimeSpeed;
                }
                estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), current_e_value), 25.0);
            }
            else
            {
                current_e_value += extruder_attr[current_extruder].isRetracted;
                *output_stream << "G1 F" << (retractionPrimeSpeed * 60) << " " << extruder_attr[current_extruder].extruderCharacter << std::setprecision(5) << current_e_value << "\n";
                currentSpeed = retractionPrimeSpeed;
                estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), current_e_value), currentSpeed);
            }
            if (getCurrentExtrudedVolume(current_extruder) > 10000.0) //According to https://github.com/Ultimaker/CuraEngine/issues/14 having more then 21m of extrusion causes inaccuracies. So reset it every 10m, just to be sure.
            {
                resetExtrusionValue();
            }
            extruder_attr[current_extruder].isRetracted = 0.0;
        }
        else if (prime_amount > 0.0)
        {
            current_e_value += prime_amount;
            *output_stream << "G1 F" << (retractionPrimeSpeed * 60) << " " << extruder_attr[current_extruder].extruderCharacter << std::setprecision(5) << current_e_value << "\n";
            currentSpeed = retractionPrimeSpeed;
            estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), current_e_value), currentSpeed);
        }
        extruder_attr[current_extruder].prime_amount = 0.0;
        current_e_value += extrusion_per_mm * diff.vSizeMM();
        *output_stream << "G1";
    }
    else
    {
        *output_stream << "G0";
                
        if (commandSocket) 
        {
            // we should send this travel as a non-retraction move
            cura::Polygons travelPoly;
            PolygonRef travel = travelPoly.newPoly();
            travel.add(Point(currentPosition.x, currentPosition.y));
            travel.add(Point(x, y));
            commandSocket->sendPolygons(extruder_attr[current_extruder].isRetracted ? MoveRetractionType : MoveCombingType, layer_nr, travelPoly, extruder_attr[current_extruder].isRetracted ? MM2INT(0.2) : MM2INT(0.1));
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
    *output_stream << "\n";
    
    currentPosition = Point3(x, y, z);
    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), current_e_value), speed);
}

void GCodeExport::writeRetraction(RetractionConfig* config, bool force)
{
    if (flavor == EGCodeFlavor::BFB)//BitsFromBytes does automatic retraction.
    {
        return;
    }
    if (extruder_attr[current_extruder].isRetracted == config->amount)
    {
        return;
    }
    if (config->amount <= 0)
    {
        return;
    }
    
    double current_extruded_volume = getCurrentExtrudedVolume(current_extruder);
    std::deque<double>& extruded_volume_at_previous_n_retractions = extruder_attr[current_extruder].extruded_volume_at_previous_n_retractions;
    while (int(extruded_volume_at_previous_n_retractions.size()) >= config->retraction_count_max) 
    {
        // extruder switch could have introduced data which falls outside the retraction window
        // also the retraction_count_max could have changed between the last retraction and this
        extruded_volume_at_previous_n_retractions.pop_back();
    }
    if (!force && config->retraction_count_max > 0 && int(extruded_volume_at_previous_n_retractions.size()) == config->retraction_count_max - 1 
        && current_extruded_volume < extruded_volume_at_previous_n_retractions.back() + config->retraction_extrusion_window * extruder_attr[current_extruder].filament_area) 
    {
        return;
    }
    extruded_volume_at_previous_n_retractions.push_front(current_extruded_volume);
    if (int(extruded_volume_at_previous_n_retractions.size()) == config->retraction_count_max) 
    {
        extruded_volume_at_previous_n_retractions.pop_back();
    }

    retractionPrimeSpeed = config->primeSpeed;
    
    double retraction_distance = config->amount;
    if (flavor == EGCodeFlavor::ULTIGCODE || flavor == EGCodeFlavor::REPRAP_VOLUMATRIC)
    {
        *output_stream << "G10\n";
        //Assume default UM2 retraction settings.
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), current_e_value - retraction_distance), 25); // TODO: hardcoded values!
    }
    else
    {
        current_e_value -= retraction_distance;
        *output_stream << "G1 F" << (config->speed * 60) << " " << extruder_attr[current_extruder].extruderCharacter << std::setprecision(5) << current_e_value << "\n";
        currentSpeed = config->speed;
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), current_e_value), currentSpeed);
    }

    extruder_attr[current_extruder].isRetracted = retraction_distance ;
    if (config->primeAmount > 0)
    {
        extruder_attr[current_extruder].prime_amount += config->primeAmount;
    }

    if (config->zHop > 0)
    {
        isZHopped = config->zHop;
        *output_stream << std::setprecision(3) << "G1 Z" << INT2MM(currentPosition.z + isZHopped) << "\n";
    }
}

void GCodeExport::writeRetraction_extruderSwitch()
{
    if (flavor == EGCodeFlavor::BFB)
    {
        if (!extruder_attr[current_extruder].isRetracted)
            *output_stream << "M103\r\n";

        extruder_attr[current_extruder].isRetracted = 1.0; // 1.0 is a stub; BFB doesn't use the actual retracted amount; retraction is performed by firmware
        return;
    }
//     resetExtrusionValue(); // TODO: why would we do this?

    
    double current_extruded_volume = getCurrentExtrudedVolume(current_extruder);
    std::deque<double>& extruded_volume_at_previous_n_retractions = extruder_attr[current_extruder].extruded_volume_at_previous_n_retractions;
    extruded_volume_at_previous_n_retractions.push_front(current_extruded_volume);

    double retraction_amount = extruder_attr[current_extruder].extruderSwitchRetraction;
    if (extruder_attr[current_extruder].isRetracted == retraction_amount)
    {
        return; 
    }
    if (flavor == EGCodeFlavor::ULTIGCODE || flavor == EGCodeFlavor::REPRAP_VOLUMATRIC)
    {
        if (extruder_attr[current_extruder].isRetracted) 
        {
            return; 
        }
        *output_stream << "G10 S1\n";
    }
    else
    {
        current_e_value -= retraction_amount;
        *output_stream << "G1 F" << (extruder_attr[current_extruder].extruderSwitchRetractionSpeed * 60) << " " 
            << extruder_attr[current_extruder].extruderCharacter << std::setprecision(5) << current_e_value << "\n";
            // the E value of the extruder switch retraction 'overwrites' the E value of the normal retraction
        currentSpeed = extruder_attr[current_extruder].extruderSwitchRetractionSpeed;
        retractionPrimeSpeed = extruder_attr[current_extruder].extruderSwitchPrimeSpeed;
    }
    extruder_attr[current_extruder].isRetracted = retraction_amount; // suppose that for UM2 the retraction amount in the firmware is equal to the provided amount
}

void GCodeExport::switchExtruder(int new_extruder)
{
    if (current_extruder == new_extruder)
        return;

    writeRetraction_extruderSwitch();

    resetExtrusionValue(); // should be called on the old extruder

    int old_extruder = current_extruder;
    current_extruder = new_extruder;

    if (flavor == EGCodeFlavor::MACH3)
    {
        resetExtrusionValue(); // also zero the E value on the new extruder
    }
    
    writeCode(extruder_attr[old_extruder].end_code.c_str());
    if (flavor == EGCodeFlavor::MAKERBOT)
    {
        *output_stream << "M135 T" << current_extruder << "\n";
    }
    else
    {
        *output_stream << "T" << current_extruder << "\n";
    }
    writeCode(extruder_attr[new_extruder].start_code.c_str());
    
    //Change the Z position so it gets re-writting again. We do not know if the switch code modified the Z position.
    currentPosition.z += 1;
}

void GCodeExport::writeCode(const char* str)
{
    *output_stream << str;
    if (flavor == EGCodeFlavor::BFB)
        *output_stream << "\r\n";
    else
        *output_stream << "\n";
}

void GCodeExport::writeFanCommand(double speed)
{
    if (currentFanSpeed == speed)
        return;
    if (speed > 0)
    {
        if (flavor == EGCodeFlavor::MAKERBOT)
            *output_stream << "M126 T0\n"; //value = speed * 255 / 100 // Makerbot cannot set fan speed...;
        else
            *output_stream << "M106 S" << (speed * 255 / 100) << "\n";
    }
    else
    {
        if (flavor == EGCodeFlavor::MAKERBOT)
            *output_stream << "M127 T0\n";
        else
            *output_stream << "M107\n";
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
    *output_stream << " S" << temperature << "\n";
    extruder_attr[extruder].currentTemperature = temperature;
}

void GCodeExport::writeBedTemperatureCommand(double temperature, bool wait)
{
    if (wait)
        *output_stream << "M190 S";
    else
        *output_stream << "M140 S";
    *output_stream << temperature << "\n";
}

void GCodeExport::finalize(int maxObjectHeight, double moveSpeed, const char* endCode)
{
    writeFanCommand(0);
    setZ(maxObjectHeight + 5000);
    writeMove(Point3(0,0,maxObjectHeight + 5000) + getPositionXY(), moveSpeed, 0);
    writeCode(endCode);
    log("Print time: %d\n", int(getTotalPrintTime()));
    log("Filament: %d\n", int(getTotalFilamentUsed(0)));
    for(int n=1; n<MAX_EXTRUDERS; n++)
        if (getTotalFilamentUsed(n) > 0)
            log("Filament%d: %d\n", n + 1, int(getTotalFilamentUsed(n)));
    output_stream->flush();
}

}//namespace cura

