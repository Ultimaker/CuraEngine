#ifndef PRINT_FEATURE
#define PRINT_FEATURE

#include <cstdint>

namespace cura
{

enum class PrintFeatureType: std::uint8_t
{
    NoneType, // used to mark unspecified jumps in polygons. libArcus depends on it
    OuterWall,
    InnerWall,
    Skin,
    Support,
    Skirt,
    Infill,
    SupportInfill,
    MoveCombing,
    MoveRetraction
};




} // namespace cura

#endif // PRINT_FEATURE
