// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_IMPLICITSHAREDDATACONTAINER_H
#define UTILS_IMPLICITSHAREDDATACONTAINER_H

namespace cura
{

template<typename DataType>
class ImplicitSharedDataContainer
{
public:
    virtual ImplicitSharedDataContainer& operator=(const ImplicitSharedDataContainer& other) noexcept
    {
        data_ = other.data_;
        return *this;
    }

    virtual ImplicitSharedDataContainer& operator=(ImplicitSharedDataContainer&& other) noexcept
    {
        data_ = std::move(other.data_);
        return *this;
    }

protected:
    ImplicitSharedDataContainer() noexcept
        : data_(std::make_shared<DataType>())
    {
    }

    ImplicitSharedDataContainer(const ImplicitSharedDataContainer& other) noexcept = default;

    ImplicitSharedDataContainer(ImplicitSharedDataContainer&& movable) noexcept = default;

    virtual ~ImplicitSharedDataContainer() = default;

    inline const DataType& getData() const noexcept
    {
        return *data_;
    }

    DataType& getData() noexcept
    {
        if (! data_.unique())
        {
            // Data is shared, make it unique so that we can modify it
            data_ = std::make_shared<DataType>(*data_);
        }

        return *data_;
    }

private:
    std::shared_ptr<DataType> data_;
};

template<typename DataType>
class ImplicitArraydDataContainer : public ImplicitSharedDataContainer<DataType>
{
public:
    inline size_t size() const noexcept
    {
        return getData().size();
    }

    inline bool empty() const noexcept
    {
        return getData().empty();
    }

    DataType::const_reference operator[](size_t index) const
    {
        return getData()[index];
    }

    void reserve(size_t min_size) noexcept
    {
        getData().reserve(min_size);
    }

    /*template<class iterator>
    ClipperLib::Path::iterator insert(ClipperLib::Path::const_iterator pos, iterator first, iterator last)
    {
        return path_->insert(pos, first, last);
    }*/

    void add(const DataType::const_reference value) noexcept
    {
        getData().push_back(value);
    }

    template<typename... Args>
    void emplace_back(Args&&... args) noexcept
    {
        getData().emplace_back(args...);
    }

    void remove(size_t index)
    {
        DataType& data = getData();
        assert(index < data.size() && index <= static_cast<size_t>(std::numeric_limits<long>::max()));
        data.erase(data.begin() + static_cast<long>(index));
    }

    /*!
     * Removes an element from the list and moves the last elements to its place. This is usually
     * faster to process because it does only requires to resize the list.
     *
     * \warning The counterpart it that it changes the order of the elements !
     */
    void removeFast(size_t index)
    {
        DataType& data = getData();
        assert(index < data.size());
        if (index < data.size() - 1)
        {
            data[index] = std::move(data.back());
        }
        data.resize(data.size() - 1);
    }

    void insert(size_t index, DataType::const_reference value)
    {
        DataType& data = getData();
        assert(index < data.size() && index <= static_cast<size_t>(std::numeric_limits<long>::max()));
        data.insert(std::next(), data.begin() + static_cast<long>(index), value);
    }

    void clear() noexcept
    {
        getData().clear();
    }

    void pop_back()
    {
        getData().pop_back();
    }

    void erase(DataType::iterator start, DataType::iterator end)
    {
        getData().erase(start, end);
    }

    DataType::iterator begin() noexcept
    {
        return getData().begin();
    }

    DataType::const_iterator begin() const noexcept
    {
        return getData().begin();
    }

    DataType::iterator end() noexcept
    {
        return getData().end();
    }

    DataType::const_iterator end() const noexcept
    {
        return getData().end();
    }

    DataType::reference front() noexcept
    {
        return getData().front();
    }

    DataType::const_reference front() const noexcept
    {
        return getData().front();
    }

    DataType::reference back() noexcept
    {
        return getData().back();
    }

    DataType::const_reference back() const noexcept
    {
        return getData().back();
    }
};

} // namespace cura

#endif // UTILS_IMPLICITSHAREDDATACONTAINER_H
