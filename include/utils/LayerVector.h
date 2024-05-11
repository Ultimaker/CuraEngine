// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_LAYER_VECTOR_H
#define UTILS_LAYER_VECTOR_H

#include <vector>

#include "raft.h"
#include "settings/types/LayerIndex.h"

namespace cura
{

/*!
 * \brief The LayerVector class mimics a std::vector but with the index being a LayerIndex, thus it can have negative
 *        values (for raft layers).
 * \note At instantiation, LayerVector will call Raft::getTotalExtraLayers() so it requires the settings to be setup.
 *       After that, it is assumed that this value will not change while the vector is used.
 * \warning It is assumed that items are inserted in layer order, and without missing items, like a std::vector would do
 */
template<class T>
class LayerVector
{
    // Required for some std calls as a container
    using value_type = T;
    using iterator = typename std::vector<value_type>::iterator;
    using const_iterator = typename std::vector<value_type>::const_iterator;
    using reference = value_type&;
    using const_reference = value_type&;

public:
    LayerVector()
    {
    }

    /*!
     * \brief Initializes the vector for use
     * \param contains_raft_layers Indicates whether this vector will contain raft layers
     * \param total_print_layers The number of print layers (not including raft layers). This is only for pre-reserving
     *                           stored data and can be safely omitted.
     * \note It is not mandatory to call this method if you do not intend to store raft layers, but still good practice
     */
    void init(bool contains_raft_layers, size_t print_layers = 0)
    {
        delta_ = contains_raft_layers ? Raft::getTotalExtraLayers() : 0;

        vector_.clear();
        if (print_layers)
        {
            vector_.reserve(print_layers + delta_);
        }
    }

    const_reference operator[](const LayerIndex& pos) const
    {
        return vector_[static_cast<size_t>(pos + delta_)];
    }

    reference operator[](const LayerIndex& pos)
    {
        return vector_[static_cast<size_t>(pos + delta_)];
    }

    const_reference at(const LayerIndex& pos) const
    {
        return vector_.at(static_cast<size_t>(pos + delta_));
    }

    reference at(const LayerIndex& pos)
    {
        return vector_.at(static_cast<size_t>(pos + delta_));
    }

    value_type get(const LayerIndex& pos) const noexcept
    {
        LayerIndex::value_type index = pos + delta_;
        if (index >= 0 && static_cast<size_t>(index) < vector_.size())
        {
            return vector_[static_cast<size_t>(index)];
        }
        return value_type{};
    }

    void pop_back()
    {
        vector_.pop_back();
    }

    void push_back(const_reference item)
    {
        vector_.push_back(item);
    }

    void push_back(T&& item)
    {
        vector_.push_back(std::move(item));
    }

    [[nodiscard]] size_t size() const
    {
        return vector_.size();
    }

    [[nodiscard]] bool empty() const
    {
        return vector_.empty();
    }

    void reserve(size_t size)
    {
        vector_.reserve(size);
    }

    void resize(size_t size)
    {
        vector_.resize(size);
    }

    void clear()
    {
        vector_.clear();
    }

    void emplace_back(auto&&... args)
    {
        vector_.emplace_back(std::forward<decltype(args)>(args)...);
    }

private:
    std::vector<value_type> vector_;
    LayerIndex::value_type delta_{ 0 };
};

} // namespace cura

#endif // UTILS_LAYER_VECTOR_H
