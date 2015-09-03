#ifndef PRINT_FEATURE
#define PRINT_FEATURE

namespace cura
{

enum class EPrintFeature : unsigned int // unused!!
{ // TODO: use in gcodePathConfigs ?
    OUTER_WALL,
    INNER_WALLS,
    INFILL,
    SKIN,
    HELPERS,
    UNCLASSIFIED,
    ENUM_COUNT
};








} // namespace cura

#endif // PRINT_FEATURE