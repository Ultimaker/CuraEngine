#ifndef SPACE_FILL_TYPE
#define SPACE_FILL_TYPE


namespace cura
{

/*!
 * Enum class enumerating the strategies with which an area can be occupied with filament
 *
 * The walls/perimeters are Polygons
 * ZigZag infill is PolyLines, and so is following mesh surface mode for non-polygon surfaces
 * Grid, Triangles and lines infill is Lines
 */
enum class SpaceFillType
{
    None,
    Polygons,
    PolyLines,
    Lines
};

} // namespace cura

#endif // SPACE_FILL_TYPE