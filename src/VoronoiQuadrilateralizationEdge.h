//Copyright (c) 2019 Ultimaker B.V.


#ifndef VORONOI_QUADRILATERALIZATION_EDGE_H
#define VORONOI_QUADRILATERALIZATION_EDGE_H


namespace arachne
{

class VoronoiQuadrilateralizationEdge
{
    using type_t = int_least16_t;
public:
    type_t type;
    static constexpr type_t NORMAL = 0; // from voronoi diagram
    static constexpr type_t EXTRA_VD = 1; // introduced to voronoi diagram in order to make the gMAT
    static constexpr type_t TRANSITION_END = 2; // introduced to voronoi diagram in order to make the gMAT
    VoronoiQuadrilateralizationEdge()
    : VoronoiQuadrilateralizationEdge(NORMAL)
    {}
    VoronoiQuadrilateralizationEdge(type_t type)
    : type(type)
    {}
};




} // namespace arachne
#endif // VORONOI_QUADRILATERALIZATION_EDGE_H
