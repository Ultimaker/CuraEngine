// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "print_operation/ContinuousExtruderMoveSequence.h"

#include <geometry/OpenPolyline.h>
#include <utils/ExtrusionLine.h>
#include <utils/types/arachne.h>

#include <range/v3/view/drop_last.hpp>
#include <range/v3/view/sliding.hpp>

#include "print_operation/ExtrusionMove.h"

namespace cura
{

ContinuousExtruderMoveSequence::ContinuousExtruderMoveSequence(bool closed, const Point3LL& start_position)
    : start_position_(start_position)
    , closed_(closed)
{
}

ContinuousExtruderMoveSequencePtr ContinuousExtruderMoveSequence::makeFrom(const ExtrusionLine& extrusion_line, const Velocity& speed)
{
    auto move_sequence = std::make_shared<ContinuousExtruderMoveSequence>(extrusion_line.is_closed_, extrusion_line.junctions_.empty() ? Point3LL() : extrusion_line.front().p_);

    for (const auto& extrusion_junctions : extrusion_line.junctions_ | ranges::views::sliding(2))
    {
        const ExtrusionJunction& start = extrusion_junctions[0];
        const ExtrusionJunction& end = extrusion_junctions[1];

        move_sequence->appendExtruderMove(std::make_shared<ExtrusionMove>(end.p_, start.w_, speed, end.w_));
    }

    return move_sequence;
}

ContinuousExtruderMoveSequencePtr ContinuousExtruderMoveSequence::makeFrom(const Polyline& polyline, const coord_t line_width, const Velocity& speed)
{
    auto move_sequence = std::make_shared<ContinuousExtruderMoveSequence>(polyline.isClosed(), polyline.empty() ? Point3LL() : polyline.front());

    for (auto iterator = polyline.beginSegments(); iterator != polyline.endSegments(); ++iterator)
    {
        move_sequence->appendExtruderMove(std::make_shared<ExtrusionMove>((*iterator).end, line_width, speed));
    }

    return move_sequence;
}

std::optional<Point3LL> ContinuousExtruderMoveSequence::findStartPosition() const
{
    return closed_ ? PrintOperationSequence::findEndPosition() : start_position_;
}

bool ContinuousExtruderMoveSequence::isClosed() const
{
    return closed_;
}

void ContinuousExtruderMoveSequence::reorderToEndWith(const std::shared_ptr<ExtruderMove>& extruder_move)
{
    if (closed_)
    {
        std::vector<std::shared_ptr<PrintOperation>> operations = getOperations();
        if (operations.size() > 1)
        {
            auto iterator = std::find(operations.begin(), operations.end(), extruder_move);
            if (iterator != operations.end())
            {
                auto next_iterator = iterator + 1;
                if (next_iterator != operations.end()) // Otherwise we already end with the given element
                {
                    std::rotate(operations.begin(), next_iterator, operations.end());
                }
            }
            else
            {
                spdlog::warn("Unable to change sequence ordering because the given move is not part of the sequence");
            }

            setOperations(operations);
        }
    }
    else
    {
        spdlog::warn("Unable to change ordering of a non-closed sequence");
    }
}

void ContinuousExtruderMoveSequence::reverse()
{
    if (closed_)
    {
#warning to be implemented, or not ?
    }
    else
    {
        std::vector<std::shared_ptr<ExtruderMove>> operations = getOperationsAs<ExtruderMove>();
        if (! operations.empty())
        {
            const Point3LL new_end_position = start_position_;
            start_position_ = operations.back()->getPosition();

            for (auto iterator = operations.rbegin(); iterator != operations.rend(); ++iterator)
            {
                auto iterator_next = std::next(iterator);
                if (iterator_next != operations.rend())
                {
                    (*iterator)->setPosition((*iterator_next)->getPosition());
                }
                else
                {
                    (*iterator)->setPosition(new_end_position);
                }
            }
        }

        std::reverse(operations.begin(), operations.end());
        setOperations(operations);
    }
}

std::shared_ptr<Polyline> ContinuousExtruderMoveSequence::calculatePolyline() const
{
    const std::vector<std::shared_ptr<ExtruderMove>> extruder_moves = getOperationsAs<ExtruderMove>();
    if (extruder_moves.empty())
    {
        return nullptr;
    }

    std::shared_ptr<Polyline> result;
    if (closed_)
    {
        constexpr bool explicitely_closed = false;
        result = std::make_shared<ClosedPolyline>(explicitely_closed);
        result->push_back(extruder_moves.back()->getPosition().toPoint2LL());

        for (const std::shared_ptr<ExtruderMove> &extruder_move : extruder_moves | ranges::views::drop_last(1))
        {
            result->push_back(extruder_move->getPosition().toPoint2LL());
        }
    }
    else
    {
        result = std::make_shared<OpenPolyline>();
        result->push_back(start_position_.toPoint2LL());

        for (const std::shared_ptr<ExtruderMove> &extruder_move : extruder_moves)
        {
            result->push_back(extruder_move->getPosition().toPoint2LL());
        }
    }

    return result;
}

void ContinuousExtruderMoveSequence::appendExtruderMove(const std::shared_ptr<ExtruderMove>& extruder_move)
{
    appendOperation(extruder_move);
}

} // namespace cura
