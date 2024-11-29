// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "operation_transformation/FeatureExtrusionScheduler.h"

#include <utils/scoring/CornerScoringCriterion.h>
#include <utils/scoring/DistanceScoringCriterion.h>
#include <utils/scoring/ExclusionAreaScoringCriterion.h>
#include <utils/scoring/RandomScoringCriterion.h>

#include <range/v3/algorithm/contains.hpp>
#include <spdlog/spdlog.h>

#include "print_operation/ContinuousExtruderMoveSequence.h"
#include "print_operation/ExtrusionMove.h"
#include "print_operation/MeshFeatureExtrusion.h"
#include "operation_transformation/BedAdhesionConstraintsGenerator.h"
#include "operation_transformation/ClosestStartPoint.h"
#include "operation_transformation/ExtruderPlanScheduler.h"
#include "operation_transformation/MeshFeaturesConstraintsGenerator.h"
#include "operation_transformation/MonotonicConstraintsGenerator.h"
#include "settings/ZSeamConfig.h"
#include "utils/scoring/BestElementFinder.h"

namespace cura
{

FeatureExtrusionScheduler::FeatureExtrusionScheduler(const FeatureExtrusionPtr& feature_extrusion, const std::vector<FeatureExtrusionPtr>& all_feature_extrusions)
    : seam_config_(getZSeamConfig(feature_extrusion))
    , extrusions_after_(makeOrderingConstraints(feature_extrusion, all_feature_extrusions))
    , moves_constraints_(makeMoveSequencesConstraints(feature_extrusion))
{
    makeStartCandidates(feature_extrusion);
    assert(! start_candidates_.empty() && "Unable to find any start candidate for the feature");
}

bool FeatureExtrusionScheduler::isFeatureProcessableNow(const FeatureExtrusionPtr& feature_extrusion) const
{
    return ! ranges::contains(extrusions_after_, feature_extrusion);
}

void FeatureExtrusionScheduler::evaluateClosestPoint(std::optional<ClosestStartPoint>& closest_point, const Point3LL& last_position) const
{
    for (const auto& [move_sequence, start_candidates] : start_candidates_)
    {
        if (! move_sequence || moveSequenceProcessableNow(move_sequence))
        {
            for (const StartCandidatePoint& start_candidate : start_candidates)
            {
                const coord_t distance_squared = (start_candidate.position - last_position).vSize2();
                if (! closest_point.has_value() || distance_squared < closest_point->distance_squared)
                {
                    closest_point = ClosestStartPoint{ distance_squared, start_candidate };
                }
            }
        }
    }
}

void FeatureExtrusionScheduler::optimizeExtruderSequencesOrder(const StartCandidatePoint& start_point, Point3LL& current_position)
{
    if (optimize_extrusion_sequences_)
    {
        const FeatureExtrusionPtr& feature = start_point.feature_extrusion;
        auto moves_sequences = feature->getOperationsAs<ContinuousExtruderMoveSequence>();
        std::vector<std::shared_ptr<PrintOperation>> ordered_sequences;
        ordered_sequences.reserve(moves_sequences.size());

        appendNextProcessedSequence(start_point, ordered_sequences, moves_sequences, current_position);

        // TODO: this can probably be optimized
        while (! moves_sequences.empty())
        {
            std::optional<ClosestStartPoint> closest_point;
            evaluateClosestPoint(closest_point, current_position);

            if (closest_point.has_value())
            {
                appendNextProcessedSequence(closest_point->point, ordered_sequences, moves_sequences, current_position);
            }
            else
            {
                spdlog::error("Unable to find a start candidate amongst move sequences, some constraints must be contradictory to each other");
                break;
            }
        }

        feature->setOperations(ordered_sequences);
    }
    else
    {
        current_position = start_point.feature_extrusion->findEndPosition().value_or(current_position);
    }
}

std::vector<FeatureExtrusionPtr>
    FeatureExtrusionScheduler::makeOrderingConstraints(const FeatureExtrusionPtr& feature_extrusion, const std::vector<FeatureExtrusionPtr>& all_feature_extrusions)
{
    std::vector<FeatureExtrusionPtr> extrusions_after;
    std::vector<std::shared_ptr<FeatureExtrusionsConstraintsGenerator>> constraints_generators;

    switch (feature_extrusion->getPrintFeatureType())
    {
    case PrintFeatureType::SkirtBrim:
        constraints_generators.push_back(std::make_shared<BedAdhesionConstraintsGenerator>());
        break;

    case PrintFeatureType::OuterWall:
    case PrintFeatureType::InnerWall:
    case PrintFeatureType::Skin:
    case PrintFeatureType::Roof:
    case PrintFeatureType::Infill:
        constraints_generators.push_back(std::make_shared<MeshFeaturesConstraintsGenerator>());
        break;

    case PrintFeatureType::Support:
    case PrintFeatureType::SupportInfill:
    case PrintFeatureType::SupportInterface:
    case PrintFeatureType::MoveCombing:
    case PrintFeatureType::MoveRetraction:
    case PrintFeatureType::PrimeTower:
    case PrintFeatureType::NoneType:
    case PrintFeatureType::NumPrintFeatureTypes:
        break;
    }

    for (const std::shared_ptr<FeatureExtrusionsConstraintsGenerator>& constraints_generator : constraints_generators)
    {
        constraints_generator->appendConstraints(feature_extrusion, all_feature_extrusions, extrusions_after);
    }

    return extrusions_after;
}

FeatureExtrusionScheduler::SequencesConstraintsMap FeatureExtrusionScheduler::makeMoveSequencesConstraints(const FeatureExtrusionPtr& feature_extrusion)
{
    SequencesConstraintsMap constraints;

    std::vector<std::shared_ptr<MoveSequencesConstraintsGenerator>> moves_constraints_generators;
    moves_constraints_generators.push_back(std::make_shared<MonotonicConstraintsGenerator>());

    for (const std::shared_ptr<MoveSequencesConstraintsGenerator>& constraints_generator : moves_constraints_generators)
    {
        constraints_generator->appendConstraints(feature_extrusion, constraints);
    }

    return constraints;
}

void FeatureExtrusionScheduler::makeStartCandidates(const FeatureExtrusionPtr& feature_extrusion)
{
    if (seam_config_)
    {
        for (const auto& move_sequence : feature_extrusion->getOperationsAs<ContinuousExtruderMoveSequence>())
        {
            // First, build a list of all the possible candidates, taking ordering constraints into account but not the seam settings and other criteria
            std::vector<StartCandidatePoint> start_candidates = makeBaseStartCandidates(feature_extrusion, move_sequence);

            // Now pre-filter the start candidates according to the seam configuration
            preFilterStartCandidates(start_candidates);

            start_candidates_[move_sequence] = start_candidates;
        }
    }
    else
    {
        // Take feature as is, do not re-order
        optimize_extrusion_sequences_ = false;
        if (const auto start_position = feature_extrusion->findStartPosition(); start_position.has_value())
        {
            start_candidates_[nullptr].emplace_back(start_position.value(), feature_extrusion, nullptr, nullptr, ChangeSequenceAction::None);
        }
        else
        {
            spdlog::error("Feature extrusion has no start position, skipping");
        }
    }
}

void FeatureExtrusionScheduler::applyMoveSequenceAction(const StartCandidatePoint& start_point)
{
    switch (start_point.action)
    {
    case ChangeSequenceAction::None:
        break;
    case ChangeSequenceAction::Reorder:
        if (start_point.move)
        {
            start_point.move_sequence->reorderToEndWith(start_point.move);
        }
        break;
    case ChangeSequenceAction::Reverse:
        start_point.move_sequence->reverse();
        break;
    }
}

bool FeatureExtrusionScheduler::moveSequenceProcessableNow(const ContinuousExtruderMoveSequencePtr& move_sequence) const
{
    return std::ranges::all_of(
        moves_constraints_,
        [&move_sequence](const auto& constraint)
        {
            return ! ranges::contains(constraint.second, move_sequence);
        });
}

void FeatureExtrusionScheduler::appendNextProcessedSequence(
    const StartCandidatePoint& start_point,
    std::vector<std::shared_ptr<PrintOperation>>& ordered_sequences,
    std::vector<ContinuousExtruderMoveSequencePtr>& moves_sequences,
    Point3LL& current_position)
{
    applyMoveSequenceAction(start_point);

    const ContinuousExtruderMoveSequencePtr& start_move_sequence = start_point.move_sequence;
    ordered_sequences.push_back(start_move_sequence);

    std::erase(moves_sequences, start_move_sequence);

    if (auto iterator = start_candidates_.find(start_move_sequence); iterator != start_candidates_.end())
    {
        start_candidates_.erase(iterator);
    }

    if (auto iterator = moves_constraints_.find(start_move_sequence); iterator != moves_constraints_.end())
    {
        moves_constraints_.erase(iterator);
    }

    std::optional<Point3LL> start_sequence_end_position = start_move_sequence->findEndPosition();
    assert(start_sequence_end_position.has_value() && "Unable to find the end position of the given start sequence");
    current_position = start_sequence_end_position.value();
}

std::vector<StartCandidatePoint> FeatureExtrusionScheduler::makeBaseStartCandidates(const FeatureExtrusionPtr& feature, const ContinuousExtruderMoveSequencePtr& move_sequence)
{
    std::vector<StartCandidatePoint> start_candidates;

    if (move_sequence->isClosed())
    {
        for (const std::shared_ptr<ExtrusionMove>& extrusion_move : move_sequence->getOperationsAs<ExtrusionMove>())
        {
            start_candidates.emplace_back(extrusion_move->getPosition(), feature, move_sequence, extrusion_move, ChangeSequenceAction::Reorder);
        }
    }
    else
    {
        if (const std::optional<Point3LL> start_position = move_sequence->findStartPosition(); start_position.has_value())
        {
            start_candidates.emplace_back(start_position.value(), feature, move_sequence, nullptr, ChangeSequenceAction::None);
        }

        if (const std::optional<Point3LL> end_position = move_sequence->findEndPosition(); end_position.has_value())
        {
            start_candidates.emplace_back(end_position.value(), feature, move_sequence, nullptr, ChangeSequenceAction::Reverse);
        }
    }

    return start_candidates;
}

void FeatureExtrusionScheduler::preFilterStartCandidates(std::vector<StartCandidatePoint>& start_candidates)
{
    // Now evaluate them according to the seam configuration
    // ########## Step 1: define the main criteria to be applied and their weights
    // Standard weight for the "main" selection criterion, depending on the selected strategy. There should be
    // exactly one calculation using this criterion.
    BestElementFinder best_candidate_finder;
    BestElementFinder::CriteriaPass main_criteria_pass;
    main_criteria_pass.outsider_delta_threshold = 0.05;

    // Indicates whether we want a single static point at the end of the selection, or a shortlist of equally good points
    bool unique_selected_point = false;

    BestElementFinder::WeighedCriterion main_criterion;

    switch (seam_config_->type_)
    {
    case EZSeamType::SHORTEST:
        // Do not set a main criterion for those, we just try to filter out very bad elements (like overhanging)
        break;

    case EZSeamType::RANDOM:
        unique_selected_point = true;
        main_criterion.criterion = std::make_shared<RandomScoringCriterion>();
        break;

    case EZSeamType::PLUGIN:
        break;

    case EZSeamType::USER_SPECIFIED:
#warning restore this
        // Use a much smaller distance divider because we want points around the forced points to be filtered out very easily
        //             constexpr double distance_divider = 1.0;
        // constexpr auto distance_type = DistanceScoringCriterion::DistanceType::Euclidian;
        // main_criterion.criterion = std::make_shared<DistanceScoringCriterion>(points, points.at(path.force_start_index_.value()), distance_type, distance_divider);
        break;

    case EZSeamType::SHARPEST_CORNER:
        unique_selected_point = true;
        main_criterion.criterion = std::make_shared<CornerScoringCriterion>(start_candidates, seam_config_->corner_pref_);
        break;
    }

    if (main_criterion.criterion)
    {
        main_criteria_pass.criteria.push_back(main_criterion);
    }

    // Second criterion with higher weight to avoid overhanging areas
    if (overhang_areas_ && ! overhang_areas_->empty())
    {
#warning give an actual area
        BestElementFinder::WeighedCriterion overhang_criterion;
        overhang_criterion.weight = 2.0;
        overhang_criterion.criterion = std::make_shared<ExclusionAreaScoringCriterion>(start_candidates, overhang_areas_);
        main_criteria_pass.criteria.push_back(overhang_criterion);
    }

    best_candidate_finder.appendCriteriaPass(main_criteria_pass);

    // ########## Step 2: add fallback passes for criteria with very similar scores (e.g. corner on a cylinder)
    if (seam_config_->type_ == EZSeamType::SHARPEST_CORNER)
    {
        std::optional<Point3LL> position_max;
        for (const StartCandidatePoint& start_candidate : start_candidates)
        {
            if (! position_max.has_value())
            {
                position_max = start_candidate.position;
            }
            else
            {
                position_max.value().x_ = std::max(position_max.value().x_, start_candidate.position.x_);
                position_max.value().y_ = std::max(position_max.value().y_, start_candidate.position.y_);
            }
        }

        { // First fallback strategy is to take points on the back-most position
            auto fallback_criterion
                = std::make_shared<DistanceScoringCriterion>(start_candidates, position_max.value_or(Point3LL()), DistanceScoringCriterion::DistanceType::YOnly);
            constexpr double outsider_delta_threshold = 0.01;
            best_candidate_finder.appendSingleCriterionPass(fallback_criterion, outsider_delta_threshold);
        }

        { // Second fallback strategy, in case we still have multiple points that are aligned on Y (e.g. cube), take the right-most point
            auto fallback_criterion
                = std::make_shared<DistanceScoringCriterion>(start_candidates, position_max.value_or(Point3LL()), DistanceScoringCriterion::DistanceType::XOnly);
            best_candidate_finder.appendSingleCriterionPass(fallback_criterion);
        }
    }

    // ########## Step 3: apply the criteria to find the vertex with the best global score
    std::vector<size_t> best_elements = best_candidate_finder.findBestElements(start_candidates.size());

    if ((unique_selected_point && ! best_elements.empty()) || best_elements.size() == 1)
    {
        start_candidates = { start_candidates[best_elements.front()] };
    }
    else
    {
        std::vector<StartCandidatePoint> new_start_candidates;
        new_start_candidates.reserve(best_elements.size());
        for (size_t best_element_index : best_elements)
        {
            new_start_candidates.push_back(start_candidates[best_element_index]);
        }
        start_candidates = std::move(new_start_candidates);
    }

#warning restore this (by adding a criterion)
    // if (! disallowed_area_for_seams.empty())
    // {
    //     best_i = pathIfZseamIsInDisallowedArea(best_i.value_or(0), path, 0);
    // }
}

std::shared_ptr<ZSeamConfig> FeatureExtrusionScheduler::getZSeamConfig(const FeatureExtrusionPtr& feature_extrusion)
{
    switch (feature_extrusion->getPrintFeatureType())
    {
    case PrintFeatureType::OuterWall:
    {
        const auto mesh_feature_extrusion = std::dynamic_pointer_cast<MeshFeatureExtrusion>(feature_extrusion);
        assert(mesh_feature_extrusion && "The outer wall feature extrusion is not a MeshFeatureExtrusion instance");
        return mesh_feature_extrusion->getMesh()->seam_config;
    }

    case PrintFeatureType::InnerWall:
    case PrintFeatureType::Skin:
    case PrintFeatureType::Roof:
    case PrintFeatureType::Support:
    case PrintFeatureType::SkirtBrim:
    case PrintFeatureType::Infill:
    case PrintFeatureType::SupportInfill:
    case PrintFeatureType::SupportInterface:
        return std::make_shared<ZSeamConfig>(EZSeamType::SHORTEST);

    case PrintFeatureType::MoveCombing:
    case PrintFeatureType::MoveRetraction:
    case PrintFeatureType::PrimeTower:
    case PrintFeatureType::NoneType:
    case PrintFeatureType::NumPrintFeatureTypes:
        break;
    }

    return nullptr;
}

} // namespace cura
