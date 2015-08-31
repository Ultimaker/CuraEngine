/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdarg.h>
#include <iomanip>

#include "gcodeExport.h"
#include "utils/logoutput.h"

namespace cura {

GCodeExport::GCodeExport()
: output_stream(&std::cout), currentPosition(0,0,0), startPosition(INT32_MIN,INT32_MIN,0)
{
    extrusion_amount = 0;
    current_extruder = 0;
    currentFanSpeed = -1;
    
    totalPrintTime = 0.0;
    
    currentSpeed = 1;
    retractionPrimeSpeed = 1;
    isRetracted = false;
    isZHopped = false;
    last_coasted_amount_mm3 = 0;
    setFlavor(EGCodeFlavor::REPRAP);
}

GCodeExport::~GCodeExport()
{
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

void GCodeExport::resetStartPosition()
{
    startPosition.x = INT32_MIN;
    startPosition.y = INT32_MIN;
}

Point GCodeExport::getStartPositionXY()
{
    return Point(startPosition.x, startPosition.y);
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

double GCodeExport::getFilamentArea(unsigned int extruder)
{
    return extruder_attr[extruder].filament_area;
}

double GCodeExport::getExtrusionAmountMM3(unsigned int extruder)
{
    if (is_volumatric)
    {
        return extrusion_amount;
    }
    else
    {
        return extrusion_amount * getFilamentArea(extruder);
    }
}


double GCodeExport::getTotalFilamentUsed(int e)
{
    if (e == current_extruder)
        return extruder_attr[e].totalFilament + getExtrusionAmountMM3(e);
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
    extrusion_amount = 0.0;
    estimateCalculator.reset();
}

void GCodeExport::updateTotalPrintTime()
{
    totalPrintTime += estimateCalculator.calculate();
    estimateCalculator.reset();
}

void GCodeExport::writeComment(std::string comment)
{
    *output_stream << ";" << comment << "\n";
}

void GCodeExport::writeTypeComment(const char* type)
{
    *output_stream << ";TYPE:" << type << "\n";
}
void GCodeExport::writeLayerComment(int layer_nr)
{
    *output_stream << ";LAYER:" << layer_nr << "\n";
}

void GCodeExport::writeLine(const char* line)
{
    *output_stream << line << "\n";
}

void GCodeExport::resetExtrusionValue()
{
    if (extrusion_amount != 0.0 && flavor != EGCodeFlavor::MAKERBOT && flavor != EGCodeFlavor::BFB)
    {
        *output_stream << "G92 " << extruder_attr[current_extruder].extruderCharacter << "0\n";
        extruder_attr[current_extruder].totalFilament += getExtrusionAmountMM3(current_extruder);
        for (unsigned int i = 0; i < extrusion_amount_at_previous_n_retractions.size(); i++)
            extrusion_amount_at_previous_n_retractions[i] -= extrusion_amount;
        extrusion_amount = 0.0;
    }
}

void GCodeExport::writeDelay(double timeAmount)
{
    *output_stream << "G4 P" << int(timeAmount * 1000) << "\n";
    totalPrintTime += timeAmount;
}

void GCodeExport::writeMove(Point p, double speed, double extrusion_mm3_per_mm)
{
    writeMove(p.X, p.Y, zPos, speed, extrusion_mm3_per_mm);
}

void GCodeExport::writeMove(Point3 p, double speed, double extrusion_mm3_per_mm)
{
    writeMove(p.x, p.y, p.z, speed, extrusion_mm3_per_mm);
}

void GCodeExport::writeMove(int x, int y, int z, double speed, double extrusion_mm3_per_mm)
{
    if (currentPosition.x == x && currentPosition.y == y && currentPosition.z == z)
        return;
    
    if (extrusion_mm3_per_mm < 0)
        logWarning("Warning! Negative extrusion move!");
    
    double extrusion_per_mm = extrusion_mm3_per_mm;
    if (!is_volumatric)
    {
        extrusion_per_mm = extrusion_mm3_per_mm / getFilamentArea(current_extruder);
    }
    
    Point gcode_pos = getGcodePos(x,y, current_extruder);

    if (flavor == EGCodeFlavor::BFB)
    {
        //For Bits From Bytes machines, we need to handle this completely differently. As they do not use E values but RPM values.
        float fspeed = speed * 60;
        float rpm = extrusion_per_mm * speed * 60;
        const float mm_per_rpm = 4.0; //All BFB machines have 4mm per RPM extrusion.
        rpm /= mm_per_rpm;
        if (rpm > 0)
        {
            if (isRetracted)
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
                isRetracted = false;
            }
            //Fix the speed by the actual RPM we are asking, because of rounding errors we cannot get all RPM values, but we have a lot more resolution in the feedrate value.
            // (Trick copied from KISSlicer, thanks Jonathan)
            fspeed *= (rpm / (roundf(rpm * 100) / 100));

            //Increase the extrusion amount to calculate the amount of filament used.
            Point3 diff = Point3(x,y,z) - getPosition();
            
            extrusion_amount += extrusion_per_mm * diff.vSizeMM();
        }else{
            //If we are not extruding, check if we still need to disable the extruder. This causes a retraction due to auto-retraction.
            if (!isRetracted)
            {
                *output_stream << "M103\r\n";
                isRetracted = true;
            }
        }
        *output_stream << std::setprecision(3) << 
            "G1 X" << INT2MM(gcode_pos.X) << 
            " Y" << INT2MM(gcode_pos.Y) << 
            " Z" << INT2MM(z) << std::setprecision(1) << " F" << fspeed << "\r\n";
    }else{
        //Normal E handling.
        
        if (extrusion_mm3_per_mm > 0.000001)
        {
            Point3 diff = Point3(x,y,z) - getPosition();
            if (isZHopped > 0)
            {
                *output_stream << std::setprecision(3) << "G1 Z" << INT2MM(currentPosition.z) << "\n";
                isZHopped = false;
            }
            extrusion_amount += (is_volumatric) ? last_coasted_amount_mm3 : last_coasted_amount_mm3 / getFilamentArea(current_extruder);   
            if (isRetracted)
            {
                if (flavor == EGCodeFlavor::ULTIGCODE || flavor == EGCodeFlavor::REPRAP_VOLUMATRIC)
                {
                    *output_stream << "G11\n";
                    //Assume default UM2 retraction settings.
                    if (last_coasted_amount_mm3 > 0)
                    {
                        *output_stream << "G1 F" << (retractionPrimeSpeed * 60) << " " << extruder_attr[current_extruder].extruderCharacter << std::setprecision(5) << extrusion_amount << "\n";
                    }
                    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), extrusion_amount), 25.0);
                }else{
                    *output_stream << "G1 F" << (retractionPrimeSpeed * 60) << " " << extruder_attr[current_extruder].extruderCharacter << std::setprecision(5) << extrusion_amount << "\n";
                    currentSpeed = retractionPrimeSpeed;
                    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), extrusion_amount), currentSpeed);
                }
                if (getExtrusionAmountMM3(current_extruder) > 10000.0) //According to https://github.com/Ultimaker/CuraEngine/issues/14 having more then 21m of extrusion causes inaccuracies. So reset it every 10m, just to be sure.
                    resetExtrusionValue();
                isRetracted = false;
            }
            else 
            {
                if (last_coasted_amount_mm3 > 0)
                {
                    *output_stream << "G1 F" << (retractionPrimeSpeed * 60) << " " << extruder_attr[current_extruder].extruderCharacter << std::setprecision(5) << extrusion_amount << "\n";
                    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), extrusion_amount), currentSpeed);
                }
            }
            last_coasted_amount_mm3 = 0;
            extrusion_amount += extrusion_per_mm * diff.vSizeMM();
            *output_stream << "G1";
        }else{
            *output_stream << "G0";
        }

        if (currentSpeed != speed)
        {
            *output_stream << " F" << (speed * 60);
            currentSpeed = speed;
        }

        *output_stream << std::setprecision(3) << 
            " X" << INT2MM(gcode_pos.X) << 
            " Y" << INT2MM(gcode_pos.Y);
        if (z != currentPosition.z)
            *output_stream << " Z" << INT2MM(z);
        if (extrusion_mm3_per_mm > 0.000001)
            *output_stream << " " << extruder_attr[current_extruder].extruderCharacter << std::setprecision(5) << extrusion_amount;
        *output_stream << "\n";
    }
    
    currentPosition = Point3(x, y, z);
    startPosition = currentPosition;
    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), extrusion_amount), speed);
}

void GCodeExport::writeRetraction(RetractionConfig* config, bool force)
{
    if (flavor == EGCodeFlavor::BFB)//BitsFromBytes does automatic retraction.
        return;
    if (isRetracted)
        return;
    if (config->amount <= 0)
        return;
    
    if (!force && config->retraction_count_max > 0 && int(extrusion_amount_at_previous_n_retractions.size()) == config->retraction_count_max - 1 
        && extrusion_amount < extrusion_amount_at_previous_n_retractions.back() + config->retraction_extrusion_window) 
        return;

    if (config->primeAmount > 0)
    {
        extrusion_amount += config->primeAmount;
    }
    retractionPrimeSpeed = config->primeSpeed;
    
    if (flavor == EGCodeFlavor::ULTIGCODE || flavor == EGCodeFlavor::REPRAP_VOLUMATRIC)
    {
        *output_stream << "G10\n";
        //Assume default UM2 retraction settings.
        double retraction_distance = 4.5;
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), extrusion_amount - retraction_distance), 25); // TODO: hardcoded values!
    }else{
        *output_stream << "G1 F" << (config->speed * 60) << " " << extruder_attr[current_extruder].extruderCharacter << std::setprecision(5) << extrusion_amount - config->amount << "\n";
        currentSpeed = config->speed;
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), extrusion_amount - config->amount), currentSpeed);
    }
    if (config->zHop > 0)
    {
        *output_stream << std::setprecision(3) << "G1 Z" << INT2MM(currentPosition.z + config->zHop) << "\n";
        isZHopped = true;
    }
    extrusion_amount_at_previous_n_retractions.push_front(extrusion_amount);
    if (int(extrusion_amount_at_previous_n_retractions.size()) == config->retraction_count_max)
    {
        extrusion_amount_at_previous_n_retractions.pop_back();
    }
    isRetracted = true;
}

void GCodeExport::writeRetraction_extruderSwitch()
{
    if (isRetracted) { return; }
        
    if (flavor == EGCodeFlavor::BFB)
    {
        if (!isRetracted)
            *output_stream << "M103\r\n";

        isRetracted = true;
        return;
    }
    resetExtrusionValue();
    if (flavor == EGCodeFlavor::ULTIGCODE || flavor == EGCodeFlavor::REPRAP_VOLUMATRIC)
    {
        *output_stream << "G10 S1\n";
    }else{
        *output_stream << "G1 F" << (extruder_attr[current_extruder].extruderSwitchRetractionSpeed * 60) << " " << extruder_attr[current_extruder].extruderCharacter << std::setprecision(5) << (extrusion_amount - extruder_attr[current_extruder].extruderSwitchRetraction) << "\n";
        currentSpeed = extruder_attr[current_extruder].extruderSwitchRetractionSpeed;
    }
    isRetracted = true;
}

void GCodeExport::switchExtruder(int new_extruder)
{
    if (current_extruder == new_extruder)
        return;
    
    if (!isRetracted) // assumes the last retraction already was an extruder switch retraction
    {
        writeRetraction_extruderSwitch();
    }
    
    int old_extruder = current_extruder;
    current_extruder = new_extruder;
    if (flavor == EGCodeFlavor::MACH3)
        resetExtrusionValue();
    isRetracted = true;
    writeCode(extruder_attr[old_extruder].end_code.c_str());
    if (flavor == EGCodeFlavor::MAKERBOT)
        *output_stream << "M135 T" << current_extruder << "\n";
    else
        *output_stream << "T" << current_extruder << "\n";
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
