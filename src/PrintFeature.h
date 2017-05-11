#ifndef PRINT_FEATURE
#define PRINT_FEATURE

namespace cura
{

enum class PrintFeatureType: unsigned char
{
    NoneType = 0, // used to mark unspecified jumps in polygons. libArcus depends on it
    OuterWall = 1,
    InnerWall = 2,
    Skin = 3,
    Support = 4,
    SkirtBrim = 5,
    Infill = 6,
    SupportInfill = 7,
    MoveCombing = 8,
    MoveRetraction = 9,
    SupportInterface = 10,
    NumPrintFeatureTypes = 11 // used to find the number of feature types in this enum
};




} // namespace cura

#endif // PRINT_FEATURE
