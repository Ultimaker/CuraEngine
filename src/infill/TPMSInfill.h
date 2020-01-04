//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "../utils/Coord_t.h"
#include "../settings/EnumSettings.h" //For infill types.
#include "../settings/types/AngleDegrees.h"
#include "../settings/types/Ratio.h"
#include "../utils/IntPoint.h"

namespace cura
{

class Polygons;

class TPMSInfill
{
public:
    TPMSInfill(const bool zig_zaggify, const coord_t line_distance, const coord_t z, const EFillResolution resolution, const Point& infill_origin, const AngleDegrees fill_angle);

    ~TPMSInfill();

    void generate(Polygons& result_lines, const Polygons& outline);
    
protected:
    const bool zig_zaggify;            //!< true if infill lines are to be connected where they meet the infill area walls
    const coord_t line_distance;       //!< distance between infill lines (pitch of TPMS curves is calculated from this to yield infill with similar material amount)
    const coord_t z;                   //!< height of the current layer
    const EFillResolution resolution;  //!< infill resolution to use
    const Point& infill_origin;        //!< point the infill is rotated around
    const double fill_angle_rads;      //!< infill rotation angle

    coord_t x_min; //!< min X coordinate of generated infill
    coord_t x_max; //!< max X coordinate of generated infill
    coord_t y_min; //!< min Y coordinate of generated infill
    coord_t y_max; //!< max Y coordinate of generated infill

    Point rotate_around_origin(const Point& point, const double rads);

private:

    virtual void generateCoordinates(Polygons& result, const Polygons& outline, const int pitch, const int step) = 0;

    virtual void generateConnections(Polygons& result, const Polygons& outline) = 0;

    virtual double pitchScaling() = 0;
};

class TPMSInfillGyroid : public TPMSInfill
{
    using TPMSInfill::TPMSInfill;

private:
    std::vector<Point> chains[2]; // [start_points[], end_points[]]
    std::vector<unsigned> connected_to[2]; // [chain_indices[], chain_indices[]]
    std::vector<int> line_numbers; // which row/column line a chain is part of

    void generateCoordinates(Polygons& result, const Polygons& outline, const int pitch, const int step);

    void generateConnections(Polygons& result, const Polygons& outline);

    double pitchScaling() { return 2.41; }
};

class TPMSInfillSchwarzP : public TPMSInfill
{
    using TPMSInfill::TPMSInfill;

private:
    std::vector<Point> connection_points;
    std::vector<unsigned> connection_ids;

    void generateCoordinates(Polygons& result, const Polygons& outline, const int pitch, const int step);

    void generateConnections(Polygons& result, const Polygons& outline);

    double pitchScaling() { return 1.94; }
};

class TPMSInfillSchwarzD : public TPMSInfill
{
    using TPMSInfill::TPMSInfill;

private:
    typedef int ConnectionId;

    std::vector<Point> connection_points;
    std::vector<ConnectionId> connection_ids;

    void generateCoordinates(Polygons& result, const Polygons& outline, const int pitch, const int step);

    void generateConnections(Polygons& result, const Polygons& outline);

    double pitchScaling() { return 1.5; }
};

} // namespace cura

