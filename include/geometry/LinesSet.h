// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_LINES_SET_H
#define GEOMETRY_LINES_SET_H

#include <vector>

#include <range/v3/view/drop.hpp>

#include "geometry/OpenLinesSet.h"
#include "geometry/Point2LL.h"

namespace cura
{

class Shape;
template<class LineType>
class LinesSet;
class OpenPolyline;

enum class CheckNonEmptyParam
{
    OnlyIfNotEmpty,
    EvenIfEmpty
};

/*!
 * \brief Base class for all geometry containers representing a set of polylines.
 * \sa https://github.com/Ultimaker/CuraEngine/wiki/Geometric-Base-Types#linesset
 */
template<class LineType>
class LinesSet
{
private:
    std::vector<LineType> lines_;

public:
    // Required for some std calls as a container
    typedef LineType value_type;

public:
    /*! \brief Builds an empty set */
    LinesSet() = default;

    /*! \brief Creates a copy of the given lines set */
    LinesSet(const LinesSet& other) = default;

    /*! \brief Constructor that takes the inner lines list from the given set */
    LinesSet(LinesSet&& other) = default;

    /*! \brief Constructor with an existing set of lines */
    LinesSet(const std::vector<LineType>& lines)
        : lines_(lines)
    {
    }

    /*! \brief Constructor that takes ownership of the data from the given set of lines */
    LinesSet(std::vector<LineType>&& lines)
        : lines_(std::move(lines))
    {
    }

    /*!
     * \brief Constructor that takes ownership of the data from the given set of lines
     * \warning This constructor is actually only defined for a LinesSet containing OpenPolyline
     *          objects, because closed ones require an additional argument
     */
    template<typename U = LineType, typename = typename std::enable_if<std::is_same<U, OpenPolyline>::value>::type>
    LinesSet(ClipperLib::Paths&& paths);

    const std::vector<LineType>& getLines() const
    {
        return lines_;
    }

    std::vector<LineType>& getLines()
    {
        return lines_;
    }

    void setLines(std::vector<LineType>&& lines)
    {
        lines_ = lines;
    }

    std::vector<LineType>::const_iterator begin() const
    {
        return lines_.begin();
    }

    std::vector<LineType>::iterator begin()
    {
        return lines_.begin();
    }

    std::vector<LineType>::const_iterator end() const
    {
        return lines_.end();
    }

    std::vector<LineType>::iterator end()
    {
        return lines_.end();
    }

    const LineType& back() const
    {
        return lines_.back();
    }

    LineType& back()
    {
        return lines_.back();
    }

    const LineType& front() const
    {
        return lines_.front();
    }

    LineType& front()
    {
        return lines_.front();
    }

    /*!
     * \brief Pushes the given line at the end of the set
     * \param checkNonEmpty Indicates whether we should check for the line to be non-empty before adding it
     */
    void push_back(const LineType& line, CheckNonEmptyParam checkNonEmpty = CheckNonEmptyParam::EvenIfEmpty);

    /*!
     * \brief Pushes the given line at the end of the set and takes ownership of the inner data
     * \param checkNonEmpty Indicates whether we should check for the line to be non-empty before adding it
     */
    void push_back(LineType&& line, CheckNonEmptyParam checkNonEmpty = CheckNonEmptyParam::EvenIfEmpty);

    /*! \brief Pushes an entier set at the end and takes ownership of the inner data */
    template<class OtherLineType>
    void push_back(LinesSet<OtherLineType>&& lines_set);

    /*! \brief Pushes an entier set at the end */
    void push_back(const LinesSet& other)
    {
        lines_.insert(lines_.end(), other.lines_.begin(), other.lines_.end());
    }

    void pop_back()
    {
        lines_.pop_back();
    }

    size_t size() const
    {
        return lines_.size();
    }

    bool empty() const
    {
        return lines_.empty();
    }

    void reserve(size_t size)
    {
        lines_.reserve(size);
    }

    void resize(size_t size)
    {
        lines_.resize(size);
    }

    void clear()
    {
        lines_.clear();
    }

    template<typename... Args>
    void emplace_back(Args&&... args)
    {
        lines_.emplace_back(args...);
    }

    std::vector<LineType>::iterator erase(std::vector<LineType>::const_iterator first, std::vector<LineType>::const_iterator last)
    {
        return lines_.erase(first, last);
    }

    LinesSet& operator=(const LinesSet& other)
    {
        lines_ = other.lines_;
        return *this;
    }

    LinesSet& operator=(LinesSet&& other)
    {
        lines_ = std::move(other.lines_);
        return *this;
    }

    LineType& operator[](size_t index)
    {
        return lines_[index];
    }

    const LineType& operator[](size_t index) const
    {
        return lines_[index];
    }

    LineType& newLine()
    {
        lines_.emplace_back();
        return lines_.back();
    }

    /*! \brief Return the amount of points in all lines */
    size_t pointCount() const;

    /*!
     * Remove a line from the list and move the last line to its place
     * \warning changes the order of the lines!
     */
    void removeAt(size_t index);

    /*! \brief Add a simple line consisting of two points */
    void addSegment(const Point2LL& from, const Point2LL& to);

    /*! \brief Get the total length of all the lines */
    coord_t length() const;

    void splitIntoSegments(OpenLinesSet& result) const;
    OpenLinesSet splitIntoSegments() const;

    /*! \brief Removes overlapping consecutive line segments which don't delimit a positive area */
    void removeDegenerateVerts();

    Shape offset(coord_t distance, ClipperLib::JoinType join_type = ClipperLib::jtMiter, double miter_limit = 1.2) const;

    /*!
     * Utility method for creating the tube (or 'donut') of a shape.
     *
     * \param inner_offset Offset relative to the original shape-outline towards the inside of the
     *        shape. Sort-of like a negative normal offset, except it's the offset part that's kept,
     *        not the shape.
     * \param outer_offset Offset relative to the original shape-outline towards the outside of the
     *        shape. Comparable to normal offset.
     * \return The resulting polygons.
     */
    Shape createTubeShape(const coord_t inner_offset, const coord_t outer_offset) const;

    void translate(const Point2LL& delta);

    /*!
     * \brief Utility method to add all the lines to a ClipperLib::Clipper object
     * \note This method needs to be public but you shouldn't need to use it from outside
     */
    void addPaths(ClipperLib::Clipper& clipper, ClipperLib::PolyType PolyTyp) const;

    /*!
     * \brief Utility method to add all the lines to a ClipperLib::ClipperOffset object
     * \note This method needs to be public but you shouldn't need to use it from outside
     */
    void addPaths(ClipperLib::ClipperOffset& clipper, ClipperLib::JoinType jointType, ClipperLib::EndType endType) const;

    /*!
     * \brief Display operator, useful for debugging/testing
     */
    template<class CharT, class TraitsT>
    friend std::basic_ostream<CharT, TraitsT>& operator<<(std::basic_ostream<CharT, TraitsT>& os, const LinesSet& lines)
    {
        os << "LinesSet[";

        for (const LineType& line : lines | ranges::views::drop(1))
        {
            os << line;
            os << ", ";
        }

        if (lines.size() > 1)
        {
            os << lines.back();
        }

        os << "]";
        return os;
    }
};

} // namespace cura

#endif // GEOMETRY_LINES_SET_H
