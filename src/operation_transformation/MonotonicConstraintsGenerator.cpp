
// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "operation_transformation/MonotonicConstraintsGenerator.h"

#include <range/v3/action/sort.hpp>
#include <range/v3/view/chunk_by.hpp>
#include <range/v3/view/sliding.hpp>

#include "geometry/Point2D.h"
#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/MeshFeatureExtrusion.h"
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
        const double same_line_distance = feature_extrusion->getLineWidth() * 0.5;

        // Lines are considered adjacent if they are less than 1 line width apart, with 10% extra play. The monotonic order is enforced if they are adjacent.
        const double max_adjacent_distance = feature_extrusion->getLineWidth() * 1.1;

        appendMonotonicConstraints(moves, angle, same_line_distance, max_adjacent_distance, constraints);
    }
}

void MonotonicConstraintsGenerator::appendMonotonicConstraints(
    const std::vector<ContinuousExtruderMoveSequencePtr>& moves,
    const AngleDegrees& angle,
    const double same_line_distance,
    const double max_adjacent_distance,
    SequencesConstraintsMap& constraints)
{
    struct ProjectedMove
    {
        ContinuousExtruderMoveSequencePtr move;
        double axial_projection;
        double radial_projection_min;
        double radial_projection_max;
    };

    std::vector<ProjectedMove> projected_moves;
    projected_moves.reserve(moves.size());

    const Point2D axial_vector(-std::cos(AngleRadians(angle)), std::sin(AngleRadians(angle)));
    const Point2D radial_vector = axial_vector.turned90CCW();

    for (const ContinuousExtruderMoveSequencePtr& move : moves)
    {
        const std::optional<Point3LL> start_position = move->findStartPosition();
        const std::optional<Point3LL> end_position = move->findEndPosition();

        if (start_position.has_value() && end_position.has_value()) [[likely]]
        {
            const double start_projection = axial_vector.dot(start_position.value().toPoint2LL());
            const double end_projection = axial_vector.dot(end_position.value().toPoint2LL());
            const double axial_projection = std::min(start_projection, end_projection); // The projection of a path is the endpoint furthest back of the two endpoints.

            const double radial_projection_start = radial_vector.dot(start_position.value().toPoint2LL());
            const double radial_projection_end = radial_vector.dot(end_position.value().toPoint2LL());
            const double radial_projection_min = std::min(radial_projection_start, radial_projection_end);
            const double radial_projection_max = std::max(radial_projection_start, radial_projection_end);

            projected_moves.emplace_back(move, axial_projection, radial_projection_min, radial_projection_max);
        }
    }

    ranges::actions::sort(
        projected_moves,
        [](const ProjectedMove& move1, const ProjectedMove& move2)
        {
            return move1.axial_projection < move2.axial_projection;
        });

    auto aligned_moves = projected_moves
                       | ranges::views::chunk_by(
                             [&same_line_distance](const ProjectedMove& move1, const ProjectedMove& move2)
                             {
                                 return move2.axial_projection - move1.axial_projection < same_line_distance;
                             });

    for (const auto& possible_adjacent_moves : aligned_moves | ranges::views::sliding(2))
    {
        // TODO: use ranges::views::adjacent instead when we have c++23 support
        const auto& moves_previous_line = *std::ranges::begin(possible_adjacent_moves);
        const auto& moves_next_line = *std::next(std::ranges::begin(possible_adjacent_moves));

        for (const ProjectedMove& move_previous_line : moves_previous_line)
        {
            for (const ProjectedMove& move_next_line : moves_next_line)
            {
                const bool adjacent = move_previous_line.radial_projection_min - move_next_line.radial_projection_max <= max_adjacent_distance
                                   && move_next_line.radial_projection_min - move_previous_line.radial_projection_max <= max_adjacent_distance;
                if (adjacent)
                {
                    constraints[move_previous_line.move].push_back(move_next_line.move);
                }
            }
        }
    }
}

} // namespace cura
