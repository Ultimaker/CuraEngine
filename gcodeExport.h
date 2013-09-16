/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef GCODEEXPORT_H
#define GCODEEXPORT_H

#if defined(__APPLE__) && defined(__MACH__)
//On MacOS the file offset functions are always 64bit.
#define off64_t off_t
#define ftello64 ftello
#define fseeko64 fseeko
#endif

class GCodeExport
{
private:
    FILE* f;
    double extrusionAmount;
    double extrusionPerMM;
    double retractionAmount;
    double extruderSwitchRetraction;
    double minimalExtrusionBeforeRetraction;
    double extrusionAmountAtPreviousRetraction;
    Point3 currentPosition;
    Point extruderOffset[16];
    int currentSpeed, retractionSpeed;
    int zPos;
    bool isRetracted;
    int extruderNr;
    int currentFanSpeed;
    int flavor;
    
    double totalFilament;
public:
    double totalPrintTime;
    
    GCodeExport()
    : currentPosition(0,0,0)
    {
        extrusionAmount = 0;
        extrusionPerMM = 0;
        retractionAmount = 4.5;
        minimalExtrusionBeforeRetraction = 0.0;
        extrusionAmountAtPreviousRetraction = -10000;
        extruderSwitchRetraction = 14.5;
        extruderNr = 0;
        currentFanSpeed = -1;
        
        totalPrintTime = 0.0;
        totalFilament = 0.0;
        
        currentSpeed = 0;
        retractionSpeed = 45;
        isRetracted = true;
        memset(extruderOffset, 0, sizeof(extruderOffset));
        f = stdout;
    }
    
    ~GCodeExport()
    {
        if (f)
            fclose(f);
    }
    
    void replaceTagInStart(const char* tag, const char* replaceValue)
    {
        off64_t oldPos = ftello64(f);
        
        char buffer[1024];
        fseeko64(f, 0, SEEK_SET);
        fread(buffer, 1024, 1, f);
        
        char* c = strstr(buffer, tag);
        memset(c, ' ', strlen(tag));
        if (c) memcpy(c, replaceValue, strlen(replaceValue));
        
        fseeko64(f, 0, SEEK_SET);
        fwrite(buffer, 1024, 1, f);
        
        fseeko64(f, oldPos, SEEK_SET);
    }
    
    void setExtruderOffset(int id, Point p)
    {
        extruderOffset[id] = p;
    }
    
    void setFlavor(int flavor)
    {
        this->flavor = flavor;
    }
    int getFlavor()
    {
        return this->flavor;
    }
    
    void setFilename(const char* filename)
    {
        f = fopen(filename, "w+");
    }
    
    bool isValid()
    {
        return f != NULL;
    }
    
    void setExtrusion(int layerThickness, int filamentDiameter, int flow)
    {
        double filamentArea = M_PI * (double(filamentDiameter) / 1000.0 / 2.0) * (double(filamentDiameter) / 1000.0 / 2.0);
        if (flavor == GCODE_FLAVOR_ULTIGCODE)//UltiGCode uses volume extrusion as E value, and thus does not need the filamentArea in the mix.
            extrusionPerMM = double(layerThickness) / 1000.0;
        else
            extrusionPerMM = double(layerThickness) / 1000.0 / filamentArea * double(flow) / 100.0;
    }
    
    void setRetractionSettings(int retractionAmount, int retractionSpeed, int extruderSwitchRetraction, int minimalExtrusionBeforeRetraction)
    {
        this->retractionAmount = double(retractionAmount) / 1000.0;
        this->retractionSpeed = retractionSpeed;
        this->extruderSwitchRetraction = double(extruderSwitchRetraction) / 1000.0;
        this->minimalExtrusionBeforeRetraction = double(minimalExtrusionBeforeRetraction) / 1000.0;
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
            extrusionAmountAtPreviousRetraction -= extrusionAmount;
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
                if (flavor == GCODE_FLAVOR_ULTIGCODE)
                {
                    fprintf(f, "G11\n");
                }else{
                    fprintf(f, "G1 F%i E%0.5lf\n", retractionSpeed * 60, extrusionAmount);
                    currentSpeed = retractionSpeed;
                }
                if (extrusionAmount > 10000.0) //According to https://github.com/Ultimaker/CuraEngine/issues/14 having more then 21m of extrusion causes inaccuracies. So reset it every 10m, just to be sure.
                    resetExtrusionValue();
                isRetracted = false;
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
        if (retractionAmount > 0 && !isRetracted && extrusionAmountAtPreviousRetraction + minimalExtrusionBeforeRetraction < extrusionAmount)
        {
            if (flavor == GCODE_FLAVOR_ULTIGCODE)
            {
                fprintf(f, "G10\n");
            }else{
                fprintf(f, "G1 F%i E%0.5lf\n", retractionSpeed * 60, extrusionAmount - retractionAmount);
                currentSpeed = retractionSpeed;
            }
            extrusionAmountAtPreviousRetraction = extrusionAmount;
            isRetracted = true;
        }
    }
    
    void switchExtruder(int newExtruder)
    {
        if (extruderNr == newExtruder)
            return;
        
        extruderNr = newExtruder;

        if (flavor == GCODE_FLAVOR_ULTIGCODE)
        {
            fprintf(f, "G10 S1\n");
        }else{
            fprintf(f, "G1 F%i E%0.4lf\n", retractionSpeed * 60, extrusionAmount - extruderSwitchRetraction);
            currentSpeed = retractionSpeed;
        }
        isRetracted = true;
        fprintf(f, "T%i\n", extruderNr);
    }
    
    void addCode(const char* str)
    {
        fprintf(f, "%s\n", str);
    }
    
    void addFanCommand(int speed)
    {
        if (currentFanSpeed == speed)
            return;
        if (speed > 0)
            fprintf(f, "M106 S%d\n", speed * 255 / 100);
        else
            fprintf(f, "M107\n");
        currentFanSpeed = speed;
    }

    int getFileSize(){
        return ftell(f);
    }
    void tellFileSize() {
        float fsize = (float) ftell(f);
        if(fsize > 1024*1024) {
            fsize /= 1024.0*1024.0;
            fprintf(stderr, "Wrote %5.1f MB.\n",fsize);
        }
        if(fsize > 1024) {
            fsize /= 1024.0;
            fprintf(stderr, "Wrote %5.1f kilobytes.\n",fsize);
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
    int extrudeSpeedFactor;
    int currentExtruder;
    int retractionMinimalDistance;
    bool forceRetraction;
    bool alwaysRetract;
    double extraTime;
    double totalPrintTime;
private:
    GCodePath* getLatestPathWithConfig(GCodePathConfig* config)
    {
        if (paths.size() > 0 && paths[paths.size()-1].config == config)
            return &paths[paths.size()-1];
        paths.push_back(GCodePath());
        GCodePath* ret = &paths[paths.size()-1];
        ret->retract = false;
        ret->config = config;
        ret->extruder = currentExtruder;
        return ret;
    }
    void forceNewPathStart()
    {
        paths.push_back(GCodePath());
        GCodePath* ret = &paths[paths.size()-1];
        ret->retract = false;
        ret->config = &moveConfig;
        ret->extruder = currentExtruder;
    }
public:
    GCodePlanner(GCodeExport& gcode, int moveSpeed, int retractionMinimalDistance)
    : gcode(gcode), moveConfig(moveSpeed, 0, "move")
    {
        lastPosition = gcode.getPositionXY();
        comb = NULL;
        extrudeSpeedFactor = 100;
        extraTime = 0.0;
        totalPrintTime = 0.0;
        forceRetraction = false;
        alwaysRetract = false;
        currentExtruder = gcode.getExtruderNr();
        this->retractionMinimalDistance = retractionMinimalDistance;
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
    
    void setAlwaysRetract(bool alwaysRetract)
    {
        this->alwaysRetract = alwaysRetract;
    }
    
    void forceRetract()
    {
        forceRetraction = true;
    }
    
    void setExtrudeSpeedFactor(int speedFactor)
    {
        if (speedFactor < 1) speedFactor = 1;
        this->extrudeSpeedFactor = speedFactor;
    }
    int getExtrudeSpeedFactor()
    {
        return this->extrudeSpeedFactor;
    }
    
    void addMove(Point p)
    {
        GCodePath* path = getLatestPathWithConfig(&moveConfig);
        if (forceRetraction)
        {
            if (!shorterThen(lastPosition - p, 1500))
            {
                path->retract = true;
            }
            forceRetraction = false;
        }else if (comb != NULL)
        {
            vector<Point> pointList;
            if (comb->calc(lastPosition, p, pointList))
            {
                for(unsigned int n=0; n<pointList.size(); n++)
                {
                    path->points.push_back(pointList[n]);
                }
            }else{
                if (!shorterThen(lastPosition - p, retractionMinimalDistance))
                    path->retract = true;
            }
        }else if (alwaysRetract)
        {
            if (!shorterThen(lastPosition - p, retractionMinimalDistance))
                path->retract = true;
        }
        path->points.push_back(p);
        lastPosition = p;
    }
    
    void addExtrusionMove(Point p, GCodePathConfig* config)
    {
        getLatestPathWithConfig(config)->points.push_back(p);
        lastPosition = p;
    }
    
    void moveInsideCombBoundary()
    {
        if (!comb || comb->checkInside(lastPosition)) return;
        Point p = lastPosition;
        if (comb->moveInside(p))
        {
            addMove(p);
            //Make sure the that any retraction happens after this move, not before it by starting a new move path.
            forceNewPathStart();
        }
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
        double travelTime = 0.0;
        double extrudeTime = 0.0;
        for(unsigned int n=0; n<paths.size(); n++)
        {
            GCodePath* path = &paths[n];
            for(unsigned int i=0; i<path->points.size(); i++)
            {
                double thisTime = vSizeMM(p0 - path->points[i]) / double(path->config->speed);
                if (path->config->lineWidth != 0)
                    extrudeTime += thisTime;
                else
                    travelTime += thisTime;
                p0 = path->points[i];
            }
        }
        double totalTime = extrudeTime + travelTime;
        if (totalTime < minTime)
        {
            double minExtrudeTime = minTime - travelTime;
            if (minExtrudeTime < 1)
                minExtrudeTime = 1;
            double factor = extrudeTime / minExtrudeTime;
            for(unsigned int n=0; n<paths.size(); n++)
            {
                GCodePath* path = &paths[n];
                if (path->config->lineWidth == 0)
                    continue;
                int speed = path->config->speed * factor;
                if (speed < minimalSpeed)
                    factor = double(minimalSpeed) / double(path->config->speed);
            }
            setExtrudeSpeedFactor(factor * 100);
            
            if (minTime - (extrudeTime / factor) - travelTime > 0.1)
            {
                //TODO: Use up this extra time (circle around the print?)
                this->extraTime = minTime - (extrudeTime / factor) - travelTime;
            }
            this->totalPrintTime = (extrudeTime / factor) + travelTime;
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
            int speed = path->config->speed;
            
            if (path->config->lineWidth != 0)// Only apply the extrudeSpeedFactor to extrusion moves
                speed = speed * extrudeSpeedFactor / 100;
            
            if (path->points.size() == 1 && path->config != &moveConfig && shorterThen(gcode.getPositionXY() - path->points[0], path->config->lineWidth * 2))
            {
                //Check for lots of small moves and combine them into one large line
                Point p0 = path->points[0];
                unsigned int i = n + 1;
                while(i < paths.size() && paths[i].points.size() == 1 && shorterThen(p0 - paths[i].points[0], path->config->lineWidth * 2))
                {
                    p0 = paths[i].points[0];
                    i ++;
                }
                if (paths[i-1].config == &moveConfig)
                    i --;
                if (i > n + 2)
                {
                    p0 = gcode.getPositionXY();
                    for(unsigned int x=n; x<i-1; x+=2)
                    {
                        int64_t oldLen = vSize(p0 - paths[x].points[0]);
                        Point newPoint = (paths[x].points[0] + paths[x+1].points[0]) / 2;
                        int64_t newLen = vSize(gcode.getPositionXY() - newPoint);
                        if (newLen > 0)
                            gcode.addMove(newPoint, speed, path->config->lineWidth * oldLen / newLen);
                        
                        p0 = paths[x+1].points[0];
                    }
                    gcode.addMove(paths[i-1].points[0], speed, path->config->lineWidth);
                    n = i - 1;
                    continue;
                }
            }
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
