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
    OnlyIfValid,
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
    using value_type = LineType;
    using iterator = typename std::vector<LineType>::iterator;
    using const_iterator = typename std::vector<LineType>::const_iterator;

    /*! \brief Builds an empty set */
    LinesSet() noexcept = default;

    virtual ~LinesSet() = default;

    /*! \brief Creates a copy of the given lines set */
    LinesSet(const LinesSet& other) = default;

    /*! \brief Constructor that takes the inner lines list from the given set */
    LinesSet(LinesSet&& other) = default;

    /*! \brief Constructor with an existing set of lines */
    explicit LinesSet(const std::vector<LineType>& lines)
        : lines_(lines)
    {
    }

    /*! \brief Constructor that takes ownership of the data from the given set of lines */
    explicit LinesSet(std::vector<LineType>&& lines)
        : lines_(std::move(lines))
    {
    }

    /*! \brief Constructor with a single existing line */
    explicit LinesSet(const LineType& line)
        : lines_({ line })
    {
    }

    /*!
     * \brief Constructor that takes ownership of the data from the given set of lines
     * \warning This constructor is actually only defined for a LinesSet containing OpenPolyline
     *          objects, because closed ones require an additional argument
     */
    template<typename U = LineType>
    requires std::is_same_v<U, OpenPolyline>
    explicit LinesSet(ClipperLib::Paths&& paths)
    {
        reserve(paths.size());
        for (ClipperLib::Path& path : paths)
        {
            lines_.emplace_back(std::move(path));
        }
    }

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

    const_iterator begin() const
    {
        return lines_.begin();
    }

    iterator begin()
    {
        return lines_.begin();
    }

    const_iterator end() const
    {
        return lines_.end();
    }

    iterator end()
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
     * \param check_non_empty Indicates whether we should check for the line to be non-empty before adding it
     */
    void push_back(const LineType& line, CheckNonEmptyParam check_non_empty = CheckNonEmptyParam::EvenIfEmpty);

    /*!
     * \brief Pushes the given line at the end of the set and takes ownership of the inner data
     * \param check_non_empty Indicates whether we should check for the line to be non-empty before adding it
     */
    void push_back(LineType&& line, CheckNonEmptyParam check_non_empty = CheckNonEmptyParam::EvenIfEmpty);

    /*! \brief Pushes an entier set at the end and takes ownership of the inner data */
    template<class OtherLineType>
    void push_back(LinesSet<OtherLineType>&& lines_set);

    /*! \brief Pushes an entire set at the end */
    void push_back(const LinesSet& other)
    {
        lines_.insert(lines_.end(), other.lines_.begin(), other.lines_.end());
    }

    void pop_back()
    {
        lines_.pop_back();
    }

    [[nodiscard]] size_t size() const
    {
        return lines_.size();
    }

    [[nodiscard]] bool empty() const
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

    void emplace_back(auto&&... args)
    {
        lines_.emplace_back(std::forward<decltype(args)>(args)...);
    }

    iterator erase(const_iterator first, const_iterator last)
    {
        return lines_.erase(first, last);
    }

    LinesSet& operator=(const LinesSet& other) = default;

    LinesSet& operator=(LinesSet&& other) noexcept = default;

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
    [[nodiscard]] size_t pointCount() const;

    /*!
     * Remove a line from the list and move the last line to its place
     * \warning changes the order of the lines!
     */
    void removeAt(size_t index);

    /*! \brief Add a simple line consisting of two points */
    void addSegment(const Point2LL& from, const Point2LL& to);

    /*! \brief Get the total length of all the lines */
    [[nodiscard]] coord_t length() const;

    void splitIntoSegments(OpenLinesSet& result) const;
    [[nodiscard]] OpenLinesSet splitIntoSegments() const;

    /*! \brief Removes overlapping consecutive line segments which don't delimit a positive area */
    void removeDegenerateVerts();

    [[nodiscard]] Shape offset(coord_t distance, ClipperLib::JoinType join_type = ClipperLib::jtMiter, double miter_limit = 1.2) const;

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
    [[nodiscard]] Shape createTubeShape(const coord_t inner_offset, const coord_t outer_offset) const;

    void translate(const Point2LL& delta);

    /*!
     * \brief Utility method to add all the lines to a ClipperLib::Clipper object
     * \note This method needs to be public but you shouldn't need to use it from outside
     */
    void addPaths(ClipperLib::Clipper& clipper, ClipperLib::PolyType poly_typ) const;

    /*!
     * \brief Utility method to add a line to a ClipperLib::Clipper object
     * \note This method needs to be public but you shouldn't need to use it from outside
     */
    template<class OtherLineLine>
    void addPath(ClipperLib::Clipper& clipper, const OtherLineLine& line, ClipperLib::PolyType poly_typ) const;

    /*!
     * \brief Utility method to add all the lines to a ClipperLib::ClipperOffset object
     * \note This method needs to be public but you shouldn't need to use it from outside
     */
    void addPaths(ClipperLib::ClipperOffset& clipper, ClipperLib::JoinType joint_type, ClipperLib::EndType end_type) const;

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

private:
    bool checkAdd(const LineType& line, CheckNonEmptyParam check_non_empty);
};

} // namespace cura

#endif // GEOMETRY_LINES_SET_H
