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
 *        values (for raft layers). It also ensures that the first element in the list is always on the very first layer.
 * \note When calling the init() method, LayerVector will call Raft::getTotalExtraLayers() so it requires the settings
 *       to be setup. This is the reason why this is not done in the constructor, and has to be called manually.
 *       After that, it is assumed that this value will not change as long as the vector is used.
 * \warning It is assumed that items are inserted in layer order, and without missing items, like a std::vector would do
 */
template<class T>
class LayerVector
{
    // Required for some std calls as a container
    using value_type = T;
    using iterator = typename std::vector<value_type>::iterator;
    using const_iterator = typename std::vector<value_type>::const_iterator;
    using reverse_iterator = typename std::vector<value_type>::reverse_iterator;
    using const_reverse_iterator = typename std::vector<value_type>::const_reverse_iterator;
    using reference = value_type&;
    using const_reference = const value_type&;
    using difference_type = typename std::vector<value_type>::difference_type;

public:
    LayerVector() = default;

    /*!
     * \brief Initializes the vector for use
     * \param contains_raft_layers Indicates whether this vector will contain raft layers
     * \param print_layers The number of print layers (not including raft layers). This is only for pre-reserving
     *                     stored data and can be safely omitted.
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

    const_iterator begin() const
    {
        return vector_.begin();
    }

    iterator begin()
    {
        return vector_.begin();
    }

    const_iterator end() const
    {
        return vector_.end();
    }

    iterator end()
    {
        return vector_.end();
    }

    const_reverse_iterator rbegin() const
    {
        return vector_.rbegin();
    }

    reverse_iterator rbegin()
    {
        return vector_.rbegin();
    }

    const_reverse_iterator rend() const
    {
        return vector_.rend();
    }

    reverse_iterator rend()
    {
        return vector_.rend();
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

    const_reference front() const
    {
        return vector_.front();
    }

    reference front()
    {
        return vector_.front();
    }

    [[nodiscard]] const_iterator iterator_at(const LayerIndex& pos) const
    {
        LayerIndex::value_type index = pos + delta_;
        if (index >= 0 && static_cast<size_t>(index) < size())
        {
            return std::next(begin(), static_cast<difference_type>(index));
        }
        return end();
    }

    [[nodiscard]] iterator iterator_at(const LayerIndex& pos)
    {
        LayerIndex::value_type index = pos + delta_;
        if (index >= 0 && static_cast<size_t>(index) < size())
        {
            return std::next(begin(), static_cast<difference_type>(index));
        }
        return end();
    }

    /*!
     * \brief Safe method to retrieve an element from the list
     * \param pos The position of the element to be retrieved
     * \return The element at the given position, or if there is none, a default-constructed value.
     */
    value_type get(const LayerIndex& pos) const noexcept
    {
        LayerIndex::value_type index = pos + delta_;
        if (index >= 0 && static_cast<size_t>(index) < size())
        {
            return vector_[static_cast<size_t>(index)];
        }
        return value_type{};
    }

    [[nodiscard]] LayerIndex getLayer(const const_iterator& it) const
    {
        return std::distance(begin(), it) - static_cast<difference_type>(delta_);
    }

    [[nodiscard]] LayerIndex getLayer(const iterator& it) const
    {
        return std::distance(begin(), it) - static_cast<difference_type>(delta_);
    }

    [[nodiscard]] LayerIndex getLayer(const const_reverse_iterator& it) const
    {
        return std::distance(it, rend()) - 1 - static_cast<difference_type>(delta_);
    }

    [[nodiscard]] LayerIndex getLayer(const reverse_iterator& it) const
    {
        return std::distance(it, rend()) - 1 - static_cast<difference_type>(delta_);
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
    size_t delta_{ 0 };
};

} // namespace cura

#endif // UTILS_LAYER_VECTOR_H
