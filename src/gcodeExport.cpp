/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdarg.h>
#include <stdio.h>

#include "gcodeExport.h"
#include "utils/logoutput.h"

namespace cura {

GCodeExport::GCodeExport()
: currentPosition(0,0,0), startPosition(INT32_MIN,INT32_MIN,0)
{
    extrusion_amount = 0;
    minimalExtrusionBeforeRetraction = 0.0;
    extrusionAmountAtPreviousRetraction = -10000;
    extruderSwitchRetraction = 14.5;
    extruderNr = 0;
    currentFanSpeed = -1;
    
    totalPrintTime = 0.0;
    for(unsigned int e=0; e<MAX_EXTRUDERS; e++)
    {
        totalFilament[e] = 0.0;
        currentTemperature[e] = 0;
    }
    
    currentSpeed = 1;
    retractionPrimeSpeed = 1;
    isRetracted = false;
    isZHopped = false;
    setFlavor(GCODE_FLAVOR_REPRAP);
    memset(extruderOffset, 0, sizeof(extruderOffset));
    f = stdout;
}

GCodeExport::~GCodeExport()
{
    if (f && f != stdout)
        fclose(f);
}

void GCodeExport::setExtruderOffset(int id, Point p)
{
    extruderOffset[id] = p;
}

Point GCodeExport::getExtruderOffset(int id)
{
    return extruderOffset[id];
}

void GCodeExport::setSwitchExtruderCode(std::string preSwitchExtruderCode, std::string postSwitchExtruderCode)
{
    this->preSwitchExtruderCode = preSwitchExtruderCode;
    this->postSwitchExtruderCode = postSwitchExtruderCode;
}

void GCodeExport::setFlavor(GCode_Flavor flavor)
{
    this->flavor = flavor;
    if (flavor == GCODE_FLAVOR_MACH3)
        for(int n=0; n<MAX_EXTRUDERS; n++)
            extruderCharacter[n] = 'A' + n;
    else
        for(int n=0; n<MAX_EXTRUDERS; n++)
            extruderCharacter[n] = 'E';
}
int GCodeExport::getFlavor()
{
    return this->flavor;
}

void GCodeExport::setFilename(const char* filename)
{
    f = fopen(filename, "w+");
}

bool GCodeExport::isOpened()
{
    return f != nullptr;
}

void GCodeExport::setRetractionSettings(int extruderSwitchRetraction, int extruderSwitchRetractionSpeed, int extruderSwitchPrimeSpeed, int minimalExtrusionBeforeRetraction)
{
    this->extruderSwitchRetraction = INT2MM(extruderSwitchRetraction);
    this->extruderSwitchRetractionSpeed = extruderSwitchRetractionSpeed;
    this->extruderSwitchPrimeSpeed = extruderSwitchPrimeSpeed;
    this->minimalExtrusionBeforeRetraction = INT2MM(minimalExtrusionBeforeRetraction);
}

void GCodeExport::setZ(int z)
{
    this->zPos = z;
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
    return startPosition.x, startPosition.y;
}

int GCodeExport::getExtruderNr()
{
    return extruderNr;
}

double GCodeExport::getTotalFilamentUsed(int e)
{
    if (e == extruderNr)
        return totalFilament[e] + extrusion_amount;
    return totalFilament[e];
}

double GCodeExport::getTotalPrintTime()
{
    return totalPrintTime;
}

void GCodeExport::updateTotalPrintTime()
{
    totalPrintTime += estimateCalculator.calculate();
    estimateCalculator.reset();
}

void GCodeExport::writeComment(const char* comment, ...)
{
    va_list args;
    va_start(args, comment);
    fprintf(f, ";");
    vfprintf(f, comment, args);
    if (flavor == GCODE_FLAVOR_BFB)
        fprintf(f, "\r\n");
    else
        fprintf(f, "\n");
    va_end(args);
}

void GCodeExport::writeLine(const char* line, ...)
{
    va_list args;
    va_start(args, line);
    vfprintf(f, line, args);
    if (flavor == GCODE_FLAVOR_BFB)
        fprintf(f, "\r\n");
    else
        fprintf(f, "\n");
    va_end(args);
}

void GCodeExport::resetExtrusionValue()
{
    if (extrusion_amount != 0.0 && flavor != GCODE_FLAVOR_MAKERBOT && flavor != GCODE_FLAVOR_BFB)
    {
        fprintf(f, "G92 %c0\n", extruderCharacter[extruderNr]);
        totalFilament[extruderNr] += extrusion_amount;
        extrusionAmountAtPreviousRetraction -= extrusion_amount;
        extrusion_amount = 0.0;
    }
}

void GCodeExport::writeDelay(double timeAmount)
{
    fprintf(f, "G4 P%d\n", int(timeAmount * 1000));
    totalPrintTime += timeAmount;
}

void GCodeExport::writeMove(Point p, int speed, double extrusion_per_mm)
{
    if (currentPosition.x == p.X && currentPosition.y == p.Y && currentPosition.z == zPos)
        return;

    if (flavor == GCODE_FLAVOR_BFB)
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
                if (currentSpeed != int(rpm * 10))
                {
                    //fprintf(f, "; %f e-per-mm %d mm-width %d mm/s\n", extrusion_per_mm, lineWidth, speed);
                    fprintf(f, "M108 S%0.1f\r\n", rpm);
                    currentSpeed = int(rpm * 10);
                }
                fprintf(f, "M%d01\r\n", extruderNr + 1);
                isRetracted = false;
            }
            //Fix the speed by the actual RPM we are asking, because of rounding errors we cannot get all RPM values, but we have a lot more resolution in the feedrate value.
            // (Trick copied from KISSlicer, thanks Jonathan)
            fspeed *= (rpm / (roundf(rpm * 100) / 100));

            //Increase the extrusion amount to calculate the amount of filament used.
            Point diff = p - getPositionXY();
            extrusion_amount += extrusion_per_mm * vSizeMM(diff);
        }else{
            //If we are not extruding, check if we still need to disable the extruder. This causes a retraction due to auto-retraction.
            if (!isRetracted)
            {
                fprintf(f, "M103\r\n");
                isRetracted = true;
            }
        }
        fprintf(f, "G1 X%0.3f Y%0.3f Z%0.3f F%0.1f\r\n", INT2MM(p.X - extruderOffset[extruderNr].X), INT2MM(p.Y - extruderOffset[extruderNr].Y), INT2MM(zPos), fspeed);
    }else{
        
        //Normal E handling.
        if (extrusion_per_mm > 0.000001)
        {
            Point diff = p - getPositionXY();
            if (isZHopped > 0)
            {
                fprintf(f, "G1 Z%0.3f\n", INT2MM(currentPosition.z));
                isZHopped = false;
            }
            if (isRetracted)
            {
                if (flavor == GCODE_FLAVOR_ULTIGCODE || flavor == GCODE_FLAVOR_REPRAP_VOLUMATRIC)
                {
                    fprintf(f, "G11\n");
                    //Assume default UM2 retraction settings.
                    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), extrusion_amount), 25.0);
                }else{
                    fprintf(f, "G1 F%i %c%0.5f\n", retractionPrimeSpeed * 60, extruderCharacter[extruderNr], extrusion_amount);
                    currentSpeed = retractionPrimeSpeed;
                    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), extrusion_amount), currentSpeed);
                }
                if (extrusion_amount > 10000.0) //According to https://github.com/Ultimaker/CuraEngine/issues/14 having more then 21m of extrusion causes inaccuracies. So reset it every 10m, just to be sure.
                    resetExtrusionValue();
                isRetracted = false;
            }
            extrusion_amount += extrusion_per_mm * vSizeMM(diff);
            fprintf(f, "G1");
        }else{
            fprintf(f, "G0");
        }

        if (currentSpeed != speed)
        {
            fprintf(f, " F%i", speed * 60);
            currentSpeed = speed;
        }

        fprintf(f, " X%0.3f Y%0.3f", INT2MM(p.X - extruderOffset[extruderNr].X), INT2MM(p.Y - extruderOffset[extruderNr].Y));
        if (zPos != currentPosition.z)
            fprintf(f, " Z%0.3f", INT2MM(zPos));
        if (extrusion_per_mm > 0.000001)
            fprintf(f, " %c%0.5f", extruderCharacter[extruderNr], extrusion_amount);
        fprintf(f, "\n");
    }
    
    currentPosition = Point3(p.X, p.Y, zPos);
    startPosition = currentPosition;
    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), extrusion_amount), speed);
}

void GCodeExport::writeRetraction(RetractionConfig* config, bool force)
{
    if (flavor == GCODE_FLAVOR_BFB)//BitsFromBytes does automatic retraction.
        return;
    if (isRetracted)
        return;
    if (!force && extrusionAmountAtPreviousRetraction + minimalExtrusionBeforeRetraction > extrusion_amount + config->amount)
        return;
    if (config->amount <= 0)
        return;

    if (config->primeAmount > 0)
        extrusion_amount += config->primeAmount;
    retractionPrimeSpeed = config->primeSpeed;
    
    if (flavor == GCODE_FLAVOR_ULTIGCODE || flavor == GCODE_FLAVOR_REPRAP_VOLUMATRIC)
    {
        fprintf(f, "G10\n");
        //Assume default UM2 retraction settings.
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), extrusion_amount - 4.5), 25);
    }else{
        fprintf(f, "G1 F%i %c%0.5f\n", config->speed * 60, extruderCharacter[extruderNr], extrusion_amount - config->amount);
        currentSpeed = config->speed;
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), extrusion_amount - config->amount), currentSpeed);
    }
    if (config->zHop > 0)
    {
        fprintf(f, "G1 Z%0.3f\n", INT2MM(currentPosition.z + config->zHop));
        isZHopped = true;
    }
    extrusionAmountAtPreviousRetraction = extrusion_amount;
    isRetracted = true;
}

void GCodeExport::switchExtruder(int newExtruder)
{
    if (extruderNr == newExtruder)
        return;
    
    if (flavor == GCODE_FLAVOR_BFB)
    {
        if (!isRetracted)
            fprintf(f, "M103\r\n");
        isRetracted = true;
        return;
    }
    
    resetExtrusionValue();
    if (flavor == GCODE_FLAVOR_ULTIGCODE || flavor == GCODE_FLAVOR_REPRAP_VOLUMATRIC)
    {
        fprintf(f, "G10 S1\n");
    }else{
        fprintf(f, "G1 F%i %c%0.5f\n", extruderSwitchRetractionSpeed * 60, extruderCharacter[extruderNr], extrusion_amount - extruderSwitchRetraction);
        currentSpeed = extruderSwitchRetractionSpeed;
    }
    extruderNr = newExtruder;
    if (flavor == GCODE_FLAVOR_MACH3)
        resetExtrusionValue();
    isRetracted = true;
    writeCode(preSwitchExtruderCode.c_str());
    if (flavor == GCODE_FLAVOR_MAKERBOT)
        fprintf(f, "M135 T%i\n", extruderNr);
    else
        fprintf(f, "T%i\n", extruderNr);
    writeCode(postSwitchExtruderCode.c_str());
}

void GCodeExport::writeCode(const char* str)
{
    fprintf(f, "%s", str);
    if (flavor == GCODE_FLAVOR_BFB)
        fprintf(f, "\r\n");
    else
        fprintf(f, "\n");
}

void GCodeExport::writeFanCommand(int speed)
{
    if (currentFanSpeed == speed)
        return;
    if (speed > 0)
    {
        if (flavor == GCODE_FLAVOR_MAKERBOT)
            fprintf(f, "M126 T0 ; value = %d\n", speed * 255 / 100);
        else
            fprintf(f, "M106 S%d\n", speed * 255 / 100);
    }
    else
    {
        if (flavor == GCODE_FLAVOR_MAKERBOT)
            fprintf(f, "M127 T0\n");
        else
            fprintf(f, "M107\n");
    }
    currentFanSpeed = speed;
}

void GCodeExport::writeTemperatureCommand(int extruder, int temperature, bool wait)
{
    if (!wait && currentTemperature[extruder] == temperature)
        return;
    
    if (wait)
        fprintf(f, "M109 ");
    else
        fprintf(f, "M104 ");
    if (extruder != extruderNr)
        fprintf(f, "T%d ", extruder);
    fprintf(f, "S%d\n", temperature);
    currentTemperature[extruder] = temperature;
}

void GCodeExport::finalize(int maxObjectHeight, int moveSpeed, const char* endCode)
{
    writeFanCommand(0);
    setZ(maxObjectHeight + 5000);
    writeMove(getPositionXY(), moveSpeed, 0);
    writeCode(endCode);
    log("Print time: %d\n", int(getTotalPrintTime()));
    log("Filament: %d\n", int(getTotalFilamentUsed(0)));
    for(int n=1; n<MAX_EXTRUDERS; n++)
        if (getTotalFilamentUsed(n) > 0)
            log("Filament%d: %d\n", n + 1, int(getTotalFilamentUsed(n)));
}

}//namespace cura
