#ifndef PRINT_FEATURE
#define PRINT_FEATURE

namespace cura
{

enum class PrintFeatureType
{
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