// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_SEGMENT_ITERATOR_H
#define GEOMETRY_SEGMENT_ITERATOR_H

#include "geometry/Point2LL.h"

namespace cura
{

enum class ConstnessType
{
    Const,
    Modifiable,
};

/*! @brief Custom iterator to loop over the segments of a polyline/polygon */
template<ConstnessType IsConst>
struct SegmentIterator
{
    /*! @brief Transitory structure used to iterate over segments within a polyline */
    struct Segment
    {
        using PointType = std::conditional_t<IsConst == ConstnessType::Const, const Point2LL, Point2LL>;

        PointType& start;
        PointType& end;
    };

    // Define type values so that std library methods can use this iterator
    using iterator_category = std::random_access_iterator_tag;
    using value_type = Segment;
    using difference_type = std::ptrdiff_t;
    using pointer = Segment*;
    using reference = Segment&;
    using source_iterator_type = std::conditional_t<IsConst == ConstnessType::Const, typename std::vector<Point2LL>::const_iterator, typename std::vector<Point2LL>::iterator>;

private:
    source_iterator_type current_pos_;
    source_iterator_type begin_;
    source_iterator_type before_end_;

public:
    SegmentIterator(source_iterator_type pos, source_iterator_type begin, source_iterator_type end)
        : current_pos_(pos)
        , begin_(begin)
        , before_end_(end != begin ? std::prev(end) : end)
    {
    }

    Segment operator*() const
    {
        if (current_pos_ == before_end_)
        {
            return Segment{ *current_pos_, *begin_ };
        }
        return Segment{ *current_pos_, *std::next(current_pos_) };
    }

    SegmentIterator& operator++()
    {
        current_pos_++;
        return *this;
    }

    bool operator==(const SegmentIterator& other) const
    {
        return current_pos_ == other.current_pos_;
    }

    bool operator!=(const SegmentIterator& other) const
    {
        return ! (*this == other);
    }

    friend difference_type operator-(const SegmentIterator<IsConst>& iterator1, const SegmentIterator<IsConst>& iterator2)
    {
        return iterator1.current_pos_ - iterator2.current_pos_;
    }
};

} // namespace cura

#endif // GEOMETRY_SEGMENT_ITERATOR_H
