#ifndef PRINT_FEATURE
#define PRINT_FEATURE

namespace cura
{

enum class PrintFeatureType : unsigned char
{
    NoneType = 0, // used to mark unspecified jumps in polygons. libArcus depends on it
    OuterWall = 1,
    InnerWall = 2,
    Skin = 3,
    Roof = 4,
    Support = 5,
    SkirtBrim = 6,
    Infill = 7,
    SupportInfill = 8,
    MoveCombing = 9,
    MoveRetraction = 10,
    SupportInterface = 11,
    PrimeTower = 12,
    NumPrintFeatureTypes = 13 // this number MUST be the last one because other modules will
                              // use this symbol to get the total number of types, which can
                              // be used to create an array or so
};


} // namespace cura

#endif // PRINT_FEATURE
