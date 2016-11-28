/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_OPTIONAL_H
#define UTILS_OPTIONAL_H

#include <algorithm> // swap
#include <type_traits> // enable_if  is_same

namespace std
{


/*!
 * optional value
 * 
 * Adaptation of std::experimental::optional
 * See http://en.cppreference.com/w/cpp/utility/optional
 * 
 * This implementation *does* allocate on the heap and is therefore not completely according to spec.
 */
template<typename T>
class optional
{
protected:
    T* instance;
public:
    optional() //!< create an optional value which is not instantiated
    : instance(nullptr)
    {
    }
    optional(const optional& other) //!< copy construction
    {
        if (other.instance)
        {
            instance = new T(*other.instance);
        }
        else
        {
            instance = nullptr;
        }
    }
    optional(optional&& other) //!< move construction
    : instance(other.instance)
    {
        other.instance = nullptr;
    }
    template<class... Args>
    constexpr explicit optional(bool not_used, Args&&... args ) //!< construct the value in place
    : instance(new T(args...))
    {
    }
    virtual ~optional() //!< simple destructor
    {
        if (instance)
        {
            delete instance;
        }
    }
    /*!
     * Assign none to this optional value.
     * 
     * \param null_ptr exactly [nullptr]
     * \return this
     */
    optional& operator=(std::nullptr_t null_ptr)
    {
        if (instance)
        {
            delete instance;
        }
        instance = nullptr;
        return *this;
    }
    optional& operator=(const optional& other)
    {
        if (instance)
        {
            delete instance;
            if (other.instance)
            {
                *instance = *other.instance;
            }
            else
            {
                instance = nullptr;
            }
        }
        else
        {
            if (other.instance)
            {
                instance = new T(*other.instance);
            }
            else
            {
                instance = nullptr;
            }
        }
        return *this;
    }
    optional& operator=(optional&& other)
    {
        instance = other.instance;
        other.instance = nullptr;
        return *this;
    }
    template<class U = T
        , typename = typename std::enable_if<std::is_assignable<T&, U>::value>::type // type U is T, T& or T&&
        >
    optional& operator=(U&& value)
    {
        if (instance)
        {
            *instance = std::forward<U>(value);
        }
        else
        {
            instance = new T(value);
        }
        return *this;
    }
    constexpr T* operator->() const
    {
        return instance;
    }
    constexpr T& operator*() const&
    {
        return *instance;
    }
    constexpr explicit operator bool() const
    {
        return instance;
    }
    constexpr T& value() const&
    {
        return *instance;
    }
    template<class U> 
    constexpr T value_or(U&& default_value) const&
    {
        return instance ? *instance : default_value;
    }
    void swap(optional& other)
    {
        std::swap(instance, other.instance);
    }
    template<class... Args>
    void emplace(Args&&... args) //!< construct a new value in place
    {
        if (instance)
        {
            *instance = T(args...);
        }
        else
        {
            instance = new T(args...);
        }
    }
};

}//namespace std
#endif//UTILS_OPTIONAL_H

