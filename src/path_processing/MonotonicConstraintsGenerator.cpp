// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_processing/MonotonicConstraintsGenerator.h"

#include <range/v3/action/sort.hpp>
#include <range/v3/view/chunk_by.hpp>

#include "path_planning/ContinuousExtruderMoveSequence.h"
#include "path_planning/MeshFeatureExtrusion.h"
#include "sliceDataStorage.h"

namespace cura
{

void MonotonicConstraintsGenerator::appendConstraints(
    const FeatureExtrusionPtr& feature_extrusion,
    std::map<ContinuousExtruderMoveSequencePtr, std::vector<ContinuousExtruderMoveSequencePtr>>& constraints) const
{
    auto mesh_feature = std::dynamic_pointer_cast<MeshFeatureExtrusion>(feature_extrusion);
    if (! mesh_feature)
    {
        return;
    }

    const std::shared_ptr<const SliceMeshStorage>& mesh = mesh_feature->getMesh();
    const std::vector<AngleDegrees>* angles = nullptr;

    if (feature_extrusion->getPrintFeatureType() == PrintFeatureType::Roof)
    {
        if (mesh->settings.get<bool>("roofing_monotonic"))
        {
            angles = &mesh->roofing_angles;
        }
    }
    else if (feature_extrusion->getPrintFeatureType() == PrintFeatureType::Skin)
    {
        if (mesh->settings.get<bool>("skin_monotonic"))
        {
            angles = &mesh->skin_angles;
        }
    }

    if (angles)
    {
        const std::vector<ContinuousExtruderMoveSequencePtr> moves = feature_extrusion->getOperationsAs<ContinuousExtruderMoveSequence>();

        const AngleDegrees angle = angles->empty() ? AngleDegrees(45) : angles->front();
#warning restore proper angle selection according to layer number
        //     roofing_angle = mesh.roofing_angles.at(gcode_layer.getLayerNr() % mesh.roofing_angles.size());
        const coord_t same_line_distance = feature_extrusion->getLineWidth() * 0.5;

        appendMonotonicConstraints(moves, angle, same_line_distance, constraints);
    }
}

void MonotonicConstraintsGenerator::appendMonotonicConstraints(
    const std::vector<ContinuousExtruderMoveSequencePtr>& moves,
    const AngleDegrees& angle,
    const coord_t same_line_distance,
    SequencesConstraintsMap& constraints) const
{
    struct ProjectedMove
    {
        ContinuousExtruderMoveSequencePtr move;
        coord_t projection_distance;
    };

    std::vector<ProjectedMove> projected_moves;
    projected_moves.reserve(moves.size());

    constexpr static coord_t monotonic_vector_resolution = 1000;
    const Point2LL monotonic_vector(-std::cos(AngleRadians(angle)) * monotonic_vector_resolution, std::sin(AngleRadians(angle)) * monotonic_vector_resolution);

    for (const ContinuousExtruderMoveSequencePtr& move : moves)
    {
        const std::optional<Point3LL> start_position = move->findStartPosition();
        const std::optional<Point3LL> end_position = move->findEndPosition();

        if (start_position.has_value() && end_position.has_value()) [[likely]]
        {
            const coord_t start_projection = dot(start_position.value().toPoint2LL(), monotonic_vector);
            const coord_t end_projection = dot(end_position.value().toPoint2LL(), monotonic_vector);
            const coord_t projection = std::min(start_projection, end_projection); // The projection of a path is the endpoint furthest back of the two endpoints.

            projected_moves.emplace_back(move, projection);
        }
    }

    ranges::actions::sort(
        projected_moves,
        [](const ProjectedMove& move1, const ProjectedMove& move2)
        {
            return move1.projection_distance < move2.projection_distance;
        });

    auto same_lines_moves = projected_moves
                          | ranges::views::chunk_by(
                                [&same_line_distance](const ProjectedMove& move1, const ProjectedMove& move2)
                                {
                                    return move2.projection_distance - move1.projection_distance < same_line_distance;
                                });

    using SameLineMovesType = std::ranges::range_value_t<decltype(same_lines_moves)>;
    std::optional<SameLineMovesType> previous_line_moves;

    for (const auto& same_line_moves : same_lines_moves)
    {
        if (previous_line_moves.has_value())
        {
            for (const ProjectedMove& move_previous_line : previous_line_moves.value())
            {
                for (const ProjectedMove& move_next_line : same_line_moves)
                {
                    constraints[move_previous_line.move].push_back(move_next_line.move);
                }
            }
        }

        previous_line_moves = same_line_moves;
    }
}

} // namespace cura
