#ifndef GCODEEXPORT_H
#define GCODEEXPORT_H

class GCodeExport
{
    FILE* f;
    double extrusionAmount;
    double extrusionPerMM;
    Point3 currentPosition;
    int moveSpeed, extrudeSpeed, currentSpeed;
public:
    GCodeExport(const char* filename)
    : currentPosition(0,0,0)
    {
        f = fopen(filename, "w");
        extrusionAmount = 0;
        extrusionPerMM = 0;
        
        moveSpeed = 150;
        extrudeSpeed = 50;
        currentSpeed = 0;
    }
    
    ~GCodeExport()
    {
        fclose(f);
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
    
    void addComment(const char* comment, ...)
    {
        va_list args;
        va_start(args, comment);
        fprintf(f, ";");
        vfprintf(f, comment, args);
        fprintf(f, "\n");
        va_end(args);
    }
    
    void addMove(Point3 p, float extrusion)
    {
        int speed;
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
        extrusionAmount += extrusion;
        if (extrusion != 0)
            fprintf(f, " E%0.4lf", extrusionAmount);
        fprintf(f, "\n");
        
        currentPosition = p;
    }
    
    void addPolygon(ClipperLib::Polygon& polygon, int z)
    {
        addMove(Point3(polygon[0].X, polygon[0].Y, z), 0.0f);
        for(unsigned int i=1; i<polygon.size(); i++)
        {
            addMove(Point3(polygon[i].X, polygon[i].Y, z), (Point(polygon[i]) - Point(polygon[i-1])).vSizeMM() * extrusionPerMM);
        }
        addMove(Point3(polygon[0].X, polygon[0].Y, z), (Point(polygon[0]) - Point(polygon[polygon.size()-1])).vSizeMM() * extrusionPerMM);
    }
    
    void addStartCode()
    {
        fprintf(f, "G21           ;metric values\n");
        fprintf(f, "G90           ;absolute positioning\n");
        fprintf(f, "M109 S210     ;Heatup to 210C\n");
        fprintf(f, "G28           ;Home\n");
        fprintf(f, "G1 Z15.0 F300 ;move the platform down 15mm\n");
        fprintf(f, "G92 E0        ;zero the extruded length\n");
        fprintf(f, "G1 F200 E3    ;extrude 3mm of feed stock\n");
        fprintf(f, "G92 E0        ;zero the extruded length again\n");
    }

    int getFileSize(){
        return ftell(f);
    }
    void tellFileSize() {
        float fsize = (float) ftell(f);
        char fmagnitude = ' ';
        if(fsize > 1024*1024) {
            fmagnitude = 'M';
            fsize /= 1024.0*1024.0;
            fprintf(stderr, "Wrote %5.1f MB.\n",fsize);
        }
        if(fsize > 1024) {
            fmagnitude = 'k';
            fsize /= 1024.0;
            fprintf(stderr, "Wrote %5.1f kilobytes.\n",fsize);
        }
    }
};

#endif//GCODEEXPORT_H
