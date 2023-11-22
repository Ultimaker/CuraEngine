#pragma once

#include <mapbox/geometry.hpp>

#include <mapbox/variant.hpp>

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

namespace mapbox {
namespace feature {

// comparator functors
struct equal_comp_shared_ptr
{

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    template <typename T>
    bool operator()(T const& lhs, T const& rhs) const
    {
        return lhs == rhs;
    }
#pragma GCC diagnostic pop

    template <typename T>
    bool operator()(std::shared_ptr<T> const& lhs, std::shared_ptr<T> const& rhs) const
    {
        if (lhs == rhs)
        {
            return true;
        }
        return *lhs == *rhs;
    }
};

struct value;

struct null_value_t
{
};

constexpr bool operator==(const null_value_t&, const null_value_t&) { return true; }
constexpr bool operator!=(const null_value_t&, const null_value_t&) { return false; }
constexpr bool operator<(const null_value_t&, const null_value_t&) { return false; }

constexpr null_value_t null_value = null_value_t();

#define DECLARE_VALUE_TYPE_ACCESOR(NAME, TYPE)        \
    TYPE* get##NAME() noexcept                        \
    {                                                 \
        return match(                                 \
            [](TYPE& val) -> TYPE* { return &val; },  \
            [](auto&) -> TYPE* { return nullptr; });  \
    }                                                 \
    const TYPE* get##NAME() const noexcept            \
    {                                                 \
        return const_cast<value*>(this)->get##NAME(); \
    }
// Multiple numeric types (uint64_t, int64_t, double) are present in order to support
// the widest possible range of JSON numbers, which do not have a maximum range.
// Implementations that produce `value`s should use that order for type preference,
// using uint64_t for positive integers, int64_t for negative integers, and double
// for non-integers and integers outside the range of 64 bits.
using value_base = mapbox::util::variant<null_value_t, bool, uint64_t, int64_t, double, std::string,
                                         std::shared_ptr<std::vector<value>>,
                                         std::shared_ptr<std::unordered_map<std::string, value>>>;

struct value : public value_base
{
    using array_type = std::vector<value>;
    using array_ptr_type = std::shared_ptr<std::vector<value>>;
    using const_array_ptr_type = std::shared_ptr<const std::vector<value>>;
    using object_type = std::unordered_map<std::string, value>;
    using object_ptr_type = std::shared_ptr<std::unordered_map<std::string, value>>;
    using const_object_ptr_type = std::shared_ptr<const std::unordered_map<std::string, value>>;

    value() : value_base(null_value) {}
    value(null_value_t) : value_base(null_value) {}
    value(bool v) : value_base(v) {}
    value(const char* c) : value_base(std::string(c)) {}
    value(std::string str) : value_base(std::move(str)) {}

    template <typename T, typename std::enable_if_t<std::is_integral<T>::value, int> = 0,
              typename std::enable_if_t<std::is_signed<T>::value, int> = 0>
    value(T t) : value_base(int64_t(t))
    {
    }

    template <typename T, typename std::enable_if_t<std::is_integral<T>::value, int> = 0,
              typename std::enable_if_t<!std::is_signed<T>::value, int> = 0>
    value(T t) : value_base(uint64_t(t))
    {
    }

    template <typename T, typename std::enable_if_t<std::is_floating_point<T>::value, int> = 0>
    value(T t) : value_base(double(t))
    {
    }
    value(array_type array) : value_base(std::make_shared<array_type>(std::forward<array_type>(array))) {}
    value(array_ptr_type array) : value_base(array) {}
    value(object_type object) : value_base(std::make_shared<object_type>(std::forward<object_type>(object))) {}
    value(object_ptr_type object) : value_base(object) {}

    bool operator==(value const& rhs) const
    {
        assert(valid() && rhs.valid());
        if (this->which() != rhs.which())
        {
            return false;
        }
        mapbox::util::detail::comparer<value, equal_comp_shared_ptr> visitor(*this);
        return visit(rhs, visitor);
    }

    explicit operator bool() const { return !is<null_value_t>(); }

    DECLARE_VALUE_TYPE_ACCESOR(Int, int64_t)
    DECLARE_VALUE_TYPE_ACCESOR(Uint, uint64_t)
    DECLARE_VALUE_TYPE_ACCESOR(Bool, bool)
    DECLARE_VALUE_TYPE_ACCESOR(Double, double)
    DECLARE_VALUE_TYPE_ACCESOR(String, std::string)

    array_ptr_type getArray() noexcept
    {
        return match(
            [](array_ptr_type& val) -> array_ptr_type { return val; },
            [](auto&) -> array_ptr_type { return nullptr; });
    }
    const_array_ptr_type getArray() const noexcept
    {
        return const_cast<value*>(this)->getArray();
    }

    object_ptr_type getObject() noexcept
    {
        return match(
            [](object_ptr_type& val) -> object_ptr_type { return val; },
            [](auto&) -> object_ptr_type { return nullptr; });
    }
    const_object_ptr_type getObject() const noexcept
    {
        return const_cast<value*>(this)->getObject();
    }
};

#undef DECLARE_VALUE_TYPE_ACCESOR

using property_map = value::object_type;

// The same considerations and requirement for numeric types apply as for `value_base`.
using identifier = mapbox::util::variant<null_value_t, uint64_t, int64_t, double, std::string>;

template <class T>
struct feature
{
    using coordinate_type = T;
    using geometry_type = mapbox::geometry::geometry<T>; // Fully qualified to avoid GCC -fpermissive error.

    geometry_type geometry;
    property_map properties;
    identifier id;

    feature()
        : geometry(),
          properties(),
          id() {}
    feature(geometry_type const& geom_)
        : geometry(geom_),
          properties(),
          id() {}
    feature(geometry_type&& geom_)
        : geometry(std::move(geom_)),
          properties(),
          id() {}
    feature(geometry_type const& geom_, property_map const& prop_)
        : geometry(geom_), properties(prop_), id() {}
    feature(geometry_type&& geom_, property_map&& prop_)
        : geometry(std::move(geom_)),
          properties(std::move(prop_)),
          id() {}
    feature(geometry_type const& geom_, property_map const& prop_, identifier const& id_)
        : geometry(geom_),
          properties(prop_),
          id(id_) {}
    feature(geometry_type&& geom_, property_map&& prop_, identifier&& id_)
        : geometry(std::move(geom_)),
          properties(std::move(prop_)),
          id(std::move(id_)) {}
};

template <class T>
constexpr bool operator==(feature<T> const& lhs, feature<T> const& rhs)
{
    return lhs.id == rhs.id && lhs.geometry == rhs.geometry && lhs.properties == rhs.properties;
}

template <class T>
constexpr bool operator!=(feature<T> const& lhs, feature<T> const& rhs)
{
    return !(lhs == rhs);
}

template <class T, template <typename...> class Cont = std::vector>
struct feature_collection : Cont<feature<T>>
{
    using coordinate_type = T;
    using feature_type = feature<T>;
    using container_type = Cont<feature_type>;
    using size_type = typename container_type::size_type;

    template <class... Args>
    feature_collection(Args&&... args) : container_type(std::forward<Args>(args)...)
    {
    }
    feature_collection(std::initializer_list<feature_type> args)
        : container_type(std::move(args)) {}
};

} // namespace feature
} // namespace mapbox
