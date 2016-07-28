#ifndef PRINT_FEATURE
#define PRINT_FEATURE

namespace cura
{

enum class PrintFeatureType: unsigned char
{
    NoneType, // used to mark unspecified jumps in polygons. libArcus depends on it
    OuterWall,
    InnerWall,
    Skin,
    Support,
    SkirtBrim,
    Infill,
    SupportInfill,
    MoveCombing,
    MoveRetraction
};




} // namespace cura

#endif // PRINT_FEATURE
