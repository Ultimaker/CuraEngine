// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "path_planning/ContinuousExtrusionMoveSequence.h"
#include "path_planning/ExtrusionMove.h"

namespace cura
{

ContinuousExtrusionMoveSequence::ContinuousExtrusionMoveSequence(bool closed, const Point3LL& start_position)
    : start_position_(start_position)
    , closed_(closed)
{
}

std::optional<Point3LL> ContinuousExtrusionMoveSequence::findStartPosition() const
{
    return closed_ ? PrintOperationSequence::findEndPosition() : start_position_;
}

coord_t ContinuousExtrusionMoveSequence::getZOffset() const
{
    return z_offset_;
}

const Ratio& ContinuousExtrusionMoveSequence::getSpeedFactor() const
{
    return speed_factor_;
}

const Ratio& ContinuousExtrusionMoveSequence::getSpeedBackPressureFactor() const
{
    return speed_back_pressure_factor_;
}

bool ContinuousExtrusionMoveSequence::isClosed() const
{
    return closed_;
}

void ContinuousExtrusionMoveSequence::reorderToEndWith(const std::shared_ptr<ExtruderMove>& extruder_move)
{
    if (closed_)
    {
        std::vector<std::shared_ptr<PrintOperation>>& operations = getOperations();
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
        }
    }
    else
    {
        spdlog::warn("Unable to change ordering of a non-closed sequence");
    }
}

void ContinuousExtrusionMoveSequence::reverse()
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

        std::reverse(getOperations().begin(), getOperations().end());
    }
}

void ContinuousExtrusionMoveSequence::appendExtruderMove(const Point3LL& position, const Ratio& line_width_ratio)
{
    appendOperation(std::make_shared<ExtrusionMove>(position, line_width_ratio));
}

} // namespace cura
