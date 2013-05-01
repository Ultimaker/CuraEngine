#ifndef GCODEEXPORT_H
#define GCODEEXPORT_H

class GCodeExport
{
private:
    FILE* f;
    double extrusionAmount;
    double extrusionPerMM;
    double retractionAmount;
    double extruderSwitchRetraction;
    Point3 currentPosition;
    Point extruderOffset[16];
    int currentSpeed, retractionSpeed;
    int zPos;
    bool isRetracted;
    int extruderNr;
    
    double totalFilament;
public:
    double totalPrintTime;
    
    GCodeExport()
    : currentPosition(0,0,0)
    {
        extrusionAmount = 0;
        extrusionPerMM = 0;
        retractionAmount = 4.5;
        extruderSwitchRetraction = 14.5;
        extruderNr = 0;
        
        totalPrintTime = 0.0;
        totalFilament = 0.0;
        
        currentSpeed = 0;
        retractionSpeed = 45;
        isRetracted = false;
        memset(extruderOffset, 0, sizeof(extruderOffset));
        extruderOffset[1] = Point(0, 22100);
    }
    
    ~GCodeExport()
    {
        if (f)
            fclose(f);
    }
    
    void setFilename(const char* filename)
    {
        f = fopen(filename, "w");
    }
    
    bool isValid()
    {
        return f != NULL;
    }
    
    void setExtrusion(int layerThickness, int filamentDiameter, int flow)
    {
        double filamentArea = M_PI * (double(filamentDiameter) / 1000.0 / 2.0) * (double(filamentDiameter) / 1000.0 / 2.0);
        extrusionPerMM = double(layerThickness) / 1000.0 / filamentArea * double(flow) / 100.0;
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
    
    Point getPositionXY()
    {
        return Point(currentPosition.x, currentPosition.y);
    }
    
    int getPositionZ()
    {
        return currentPosition.z;
    }

    int getExtruderNr()
    {
        return extruderNr;
    }
    
    double getTotalFilamentUsed()
    {
        return totalFilament + extrusionAmount;
    }

    double getTotalPrintTime()
    {
        return totalPrintTime;
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
    
    void resetExtrusionValue()
    {
        if (extrusionAmount != 0.0)
        {
            fprintf(f, "G92 E0\n");
            totalFilament += extrusionAmount;
            extrusionAmount = 0.0;
        }
    }
    
    void addDelay(double timeAmount)
    {
        fprintf(f, "G4 P%d\n", int(timeAmount * 1000));
    }
    
    void addMove(Point p, int speed, int lineWidth)
    {
        if (lineWidth != 0)
        {
            Point diff = p - getPositionXY();
            if (isRetracted)
            {
                fprintf(f, "G1 F%i E%0.5lf\n", retractionSpeed * 60, extrusionAmount);
                currentSpeed = retractionSpeed;
                isRetracted = false;
            }else if (shorterThen(diff, 50)){
                return;
            }
            extrusionAmount += extrusionPerMM * double(lineWidth) / 1000.0 * vSizeMM(diff);
            fprintf(f, "G1");
        }else{
            fprintf(f, "G0");
        }
        
        if (currentSpeed != speed)
        {
            fprintf(f, " F%i", speed * 60);
            currentSpeed = speed;
        }
        fprintf(f, " X%0.2f Y%0.2f", float(p.X - extruderOffset[extruderNr].X)/1000, float(p.Y - extruderOffset[extruderNr].Y)/1000);
        if (zPos != currentPosition.z)
            fprintf(f, " Z%0.2f", float(zPos)/1000);
        if (lineWidth != 0)
            fprintf(f, " E%0.5lf", extrusionAmount);
        fprintf(f, "\n");
        
        currentPosition = Point3(p.X, p.Y, zPos);
    }
    
    void addRetraction()
    {
        if (retractionAmount > 0 && !isRetracted)
        {
            fprintf(f, "G1 F%i E%0.5lf\n", retractionSpeed * 60, extrusionAmount - retractionAmount);
            currentSpeed = retractionSpeed;
            isRetracted = true;
        }
    }
    
    void switchExtruder(int newExtruder)
    {
        if (extruderNr == newExtruder)
            return;
        
        extruderNr = newExtruder;

        fprintf(f, "G1 F%i E%0.4lf\n", retractionSpeed * 60, extrusionAmount - extruderSwitchRetraction);
        currentSpeed = retractionSpeed;
        isRetracted = true;
        fprintf(f, "T%i\n", extruderNr);
    }
    
    void addCode(const char* str)
    {
        fprintf(f, "%s\n", str);
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

class GCodePathConfig
{
public:
    int speed;
    int lineWidth;
    const char* name;
    
    GCodePathConfig(int speed, int lineWidth, const char* name)
    : speed(speed), lineWidth(lineWidth), name(name)
    {
    }
};

class GCodePath
{
public:
    GCodePathConfig* config;
    bool retract;
    int extruder;
    vector<Point> points;
};

class GCodePlanner
{
private:
    GCodeExport& gcode;
    
    Point lastPosition;
    vector<GCodePath> paths;
    Comb* comb;
    
    GCodePathConfig moveConfig;
    int speedFactor;
    int currentExtruder;
    double extraTime;
    double totalPrintTime;
private:
    GCodePath* getLatestPathWithConfig(GCodePathConfig* config)
    {
        if (paths.size() > 0 && paths[paths.size()-1].config == config)
        {
            return &paths[paths.size()-1];
        }
        paths.push_back(GCodePath());
        GCodePath* ret = &paths[paths.size()-1];
        ret->retract = false;
        ret->config = config;
        ret->extruder = currentExtruder;
        return ret;
    }
public:
    GCodePlanner(GCodeExport& gcode, int moveSpeed)
    : gcode(gcode), moveConfig(moveSpeed, 0, "move")
    {
        lastPosition = gcode.getPositionXY();
        comb = NULL;
        speedFactor = 100;
        extraTime = 0.0;
        totalPrintTime = 0.0;
        currentExtruder = gcode.getExtruderNr();
    }
    ~GCodePlanner()
    {
        if (comb)
            delete comb;
    }
    void setExtruder(int extruder)
    {
        currentExtruder = extruder;
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
    
    void setSpeedFactor(int speedFactor)
    {
        if (speedFactor < 1) speedFactor = 1;
        this->speedFactor = speedFactor;
    }
    
    void addMove(Point p)
    {
        GCodePath* path = getLatestPathWithConfig(&moveConfig);
        if (comb != NULL)
        {
            vector<Point> pointList;
            if (comb->calc(lastPosition, p, pointList))
            {
                for(unsigned int n=0; n<pointList.size(); n++)
                {
                    path->points.push_back(pointList[n]);
                }
            }else{
                path->retract = true;
            }
        }
        path->points.push_back(p);
        lastPosition = p;
    }
    
    void addExtrusionMove(Point p, GCodePathConfig* config)
    {
        getLatestPathWithConfig(config)->points.push_back(p);
        lastPosition = p;
    }

    void addPolygon(ClipperLib::Polygon& polygon, int startIdx, GCodePathConfig* config)
    {
        Point p0 = polygon[startIdx];
        addMove(p0);
        for(unsigned int i=1; i<polygon.size(); i++)
        {
            Point p1 = polygon[(startIdx + i) % polygon.size()];
            addExtrusionMove(p1, config);
            p0 = p1;
        }
        if (polygon.size() > 2)
            addExtrusionMove(polygon[startIdx], config);
    }

    void addPolygonsByOptimizer(Polygons& polygons, GCodePathConfig* config)
    {
        PathOptimizer orderOptimizer(lastPosition);
        for(unsigned int i=0;i<polygons.size();i++)
            orderOptimizer.addPolygon(polygons[i]);
        orderOptimizer.optimize();
        for(unsigned int i=0;i<orderOptimizer.polyOrder.size();i++)
        {
            int nr = orderOptimizer.polyOrder[i];
            addPolygon(polygons[nr], orderOptimizer.polyStart[nr], config);
        }
    }
    
    void forceMinimalLayerTime(double minTime, int minimalSpeed)
    {
        Point p0 = gcode.getPositionXY();
        double totalTime = 0.0;
        for(unsigned int n=0; n<paths.size(); n++)
        {
            GCodePath* path = &paths[n];
            for(unsigned int i=0; i<path->points.size(); i++)
            {
                totalTime += vSizeMM(p0 - path->points[i]) / double(path->config->speed);
                p0 = path->points[i];
            }
        }
        if (totalTime < minTime)
        {
            double factor = totalTime / minTime;
            for(unsigned int n=0; n<paths.size(); n++)
            {
                GCodePath* path = &paths[n];
                int speed = path->config->speed * factor;
                if (speed < minimalSpeed)
                    factor = double(minimalSpeed) / double(path->config->speed);
            }
            setSpeedFactor(factor * 100);
            
            if (minTime - (totalTime / factor) > 0.1)
            {
                //TODO: Use up this extra time (circle around the print?)
                this->extraTime = minTime - (totalTime / factor);
            }
            this->totalPrintTime = totalTime * factor;
        }else{
            this->totalPrintTime = totalTime;
        }
    }
    
    void writeGCode(bool liftHeadIfNeeded)
    {
        GCodePathConfig* lastConfig = NULL;
        int extruder = gcode.getExtruderNr();
        for(unsigned int n=0; n<paths.size(); n++)
        {
            GCodePath* path = &paths[n];
            if (extruder != path->extruder)
            {
                extruder = path->extruder;
                gcode.switchExtruder(extruder);
            }else if (path->retract)
            {
                gcode.addRetraction();
            }
            if (path->config != &moveConfig && lastConfig != path->config)
            {
                gcode.addComment("TYPE:%s", path->config->name);
                lastConfig = path->config;
            }
            int speed = path->config->speed * speedFactor / 100;
            for(unsigned int i=0; i<path->points.size(); i++)
            {
                gcode.addMove(path->points[i], speed, path->config->lineWidth);
            }
        }
        
        gcode.totalPrintTime += this->totalPrintTime;
        if (liftHeadIfNeeded && extraTime > 0.0)
        {
            gcode.totalPrintTime += extraTime;
            
            gcode.addComment("Small layer, adding delay of %f", extraTime);
            gcode.addRetraction();
            gcode.setZ(gcode.getPositionZ() + 3000);
            gcode.addMove(gcode.getPositionXY(), moveConfig.speed, 0);
            gcode.addMove(gcode.getPositionXY() - Point(-20000, 0), moveConfig.speed, 0);
            gcode.addDelay(extraTime);
        }
    }
};

#endif//GCODEEXPORT_H
