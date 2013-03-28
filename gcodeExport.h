#ifndef GCODEEXPORT_H
#define GCODEEXPORT_H

class GCodeExport
{
private:
    FILE* f;
    double extrusionAmount;
    double extrusionPerMM;
    double retractionAmount;
    Point3 currentPosition;
    int moveSpeed, extrudeSpeed, currentSpeed, retractionSpeed;
    int zPos;
    bool isRetracted;
    Comb* comb;

public:
    GCodeExport()
    : currentPosition(0,0,0)
    {
        extrusionAmount = 0;
        extrusionPerMM = 0;
        retractionAmount = 4.5;
        
        moveSpeed = 150;
        extrudeSpeed = 50;
        currentSpeed = 0;
        retractionSpeed = 45;
        isRetracted = false;
        comb = NULL;
    }
    
    ~GCodeExport()
    {
        if (f)
            fclose(f);
        if (comb)
            delete comb;
    }
    
    void setFilename(const char* filename)
    {
        f = fopen(filename, "w");
    }
    
    bool isValid()
    {
        return f != NULL;
    }
    
    void setExtrusion(int layerThickness, int lineWidth, int filamentDiameter)
    {
        double filamentArea = M_PI * (double(filamentDiameter) / 1000.0 / 2.0) * (double(filamentDiameter) / 1000.0 / 2.0);
        extrusionPerMM = double(layerThickness) / 1000.0 * double(lineWidth) / 1000.0 / filamentArea;
    }
    
    void setSpeeds(int moveSpeed, int extrudeSpeed)
    {
        this->moveSpeed = moveSpeed;
        this->extrudeSpeed = extrudeSpeed;
    }

    void setRetractionSettings(int retractionAmount, int retractionSpeed)
    {
        this->retractionAmount = double(retractionAmount) / 1000.0;
        this->retractionSpeed = retractionSpeed;
    }
    
    void setZ(int z)
    {
        this->zPos = z;
    }
    
    void setCombBoundary(Polygons* polygons)
    {
        if (comb)
            delete comb;
        if (polygons)
            comb = new Comb(*polygons);
        else
            comb = NULL;
    }
    
    Point getPositionXY()
    {
        return Point(currentPosition.x, currentPosition.y);
    }
    
    void addComment(const char* comment, ...)
    {
        va_list args;
        va_start(args, comment);
        fprintf(f, ";");
        vfprintf(f, comment, args);
        fprintf(f, "\n");
        va_end(args);
    }

    void addLine(const char* line, ...)
    {
        va_list args;
        va_start(args, line);
        vfprintf(f, line, args);
        fprintf(f, "\n");
        va_end(args);
    }
    
    void addMove(Point3 p, double extrusion)
    {
        int speed;
        if (extrusion == 0 && !isRetracted && comb != NULL)
        {
            vector<Point> pointList;
            if (comb->calc(Point(currentPosition.x, currentPosition.y), Point(p.x, p.y), pointList))
            {
                //Fake retraction so the addMove code doesn't try to comb again.
                isRetracted = true;
                for(unsigned int n=0; n<pointList.size(); n++)
                {
                    addMove(Point3(pointList[n].X, pointList[n].Y, p.z), 0.0);
                }
                isRetracted = false;
            }else{
                addRetraction();
            }
        }
        if (extrusion != 0 && isRetracted)
        {
            fprintf(f, "G1 F%i E%0.4lf\n", retractionSpeed * 60, extrusionAmount);
            isRetracted = false;
        }
        //if ((p - currentPosition).testLength(200))
        //    return;
        extrusionAmount += extrusion;
        if (extrusion != 0)
        {
            fprintf(f, "G1");
            speed = extrudeSpeed;
        }else{
            fprintf(f, "G0");
            speed = moveSpeed;
        }
        
        if (currentSpeed != speed)
        {
            fprintf(f, " F%i", speed * 60);
            currentSpeed = speed;
        }
        fprintf(f, " X%0.2f Y%0.2f", float(p.x)/1000, float(p.y)/1000);
        if (p.z != currentPosition.z)
            fprintf(f, " Z%0.2f", float(p.z)/1000);
        if (extrusion != 0)
            fprintf(f, " E%0.4lf", extrusionAmount);
        fprintf(f, "\n");
        
        currentPosition = p;
    }
    
    void addPolygon(ClipperLib::Polygon& polygon, int startIdx)
    {
        Point p0 = polygon[startIdx];
        addMove(Point3(p0.X, p0.Y, zPos), 0.0f);
        for(unsigned int i=1; i<polygon.size(); i++)
        {
            Point p1 = polygon[(startIdx + i) % polygon.size()];
            addMove(Point3(p1.X, p1.Y, zPos), vSizeMM(p1 - p0) * extrusionPerMM);
            p0 = p1;
        }
        if (polygon.size() > 2)
            addMove(Point3(polygon[startIdx].X, polygon[startIdx].Y, zPos), vSizeMM(polygon[startIdx] - p0) * extrusionPerMM);
    }
    
    void addPolygonsByOptimizer(Polygons& polygons)
    {
        PathOptimizer orderOptimizer(getPositionXY());
        for(unsigned int i=0;i<polygons.size();i++)
            orderOptimizer.addPolygon(polygons[i]);
        orderOptimizer.optimize();
        for(unsigned int i=0;i<orderOptimizer.polyOrder.size();i++)
        {
            int nr = orderOptimizer.polyOrder[i];
            addPolygon(polygons[nr], orderOptimizer.polyStart[nr]);
        }
    }
    
    void addRetraction()
    {
        if (retractionAmount > 0)
        {
            fprintf(f, "G1 F%i E%0.4lf\n", retractionSpeed * 60, extrusionAmount - retractionAmount);
            currentSpeed = retractionSpeed;
            isRetracted = true;
        }
    }
    
    void addStartCode()
    {
        fprintf(f, "M109 S210     ;Heatup to 210C\n");
        fprintf(f, "G21           ;metric values\n");
        fprintf(f, "G90           ;absolute positioning\n");
        fprintf(f, "G28           ;Home\n");
        fprintf(f, "G1 Z15.0 F300 ;move the platform down 15mm\n");
        fprintf(f, "G92 E0        ;zero the extruded length\n");
        fprintf(f, "G1 F200 E5    ;extrude 5mm of feed stock\n");
        fprintf(f, "G92 E0        ;zero the extruded length again\n");
    }
    void addEndCode()
    {
        fprintf(f, "M104 S0                     ;extruder heater off\n");
        fprintf(f, "M140 S0                     ;heated bed heater off (if you have it)\n");
        fprintf(f, "G91                            ;relative positioning\n");
        fprintf(f, "G1 E-1 F300                    ;retract the filament a bit before lifting the nozzle, to release some of the pressure\n");
        fprintf(f, "G1 Z+0.5 E-5 X-20 Y-20 F9000   ;move Z up a bit and retract filament even more\n");
        fprintf(f, "G28 X0 Y0                      ;move X/Y to min endstops, so the head is out of the way\n");
        fprintf(f, "M84                         ;steppers off\n");
        fprintf(f, "G90                         ;absolute positioning\n");
    }
    void addFanCommand(int speed)
    {
        if (speed > 0)
            fprintf(f, "M106 S%d\n", speed);
        else
            fprintf(f, "M107\n");
    }

    int getFileSize(){
        return ftell(f);
    }
    void tellFileSize() {
        float fsize = (float) ftell(f);
        if(fsize > 1024*1024) {
            fsize /= 1024.0*1024.0;
            fprintf(stdout, "Wrote %5.1f MB.\n",fsize);
        }
        if(fsize > 1024) {
            fsize /= 1024.0;
            fprintf(stdout, "Wrote %5.1f kilobytes.\n",fsize);
        }
    }
};

#endif//GCODEEXPORT_H
