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
    Support = 4,
    SkirtBrim = 5,
    Infill = 6,
    SupportInfill = 7,
    MoveUnretracted = 8,
    MoveRetracted = 9,
    SupportInterface = 10,
    PrimeTower = 11,
    MoveWhileRetracting = 12,
    MoveWhileUnretracting = 13,
    NumPrintFeatureTypes = 14 // this number MUST be the last one because other modules will
                              // use this symbol to get the total number of types, which can
                              // be used to create an array or so
};


} // namespace cura

#endif // PRINT_FEATURE
