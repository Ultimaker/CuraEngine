/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_LAZY_INITIALIZATION_H
#define UTILS_LAZY_INITIALIZATION_H

#include <functional> // bind, function

#include "optional.h"

namespace cura
{

/*!
 * Class for initializing an object only when it's requested
 * 
 * Credits to Johannes Goller
 * 
 * \tparam T The type of the object to instantiate lazily
 * \tparam Args The types of the arguments to the constructor or constructor function object
 */
template <typename T, typename... Args>
class LazyInitialization : public std::optional<T>
{
public:

    using bind_type = decltype(std::bind(std::declval<std::function<void(Args...)>>(),std::declval<Args>()...)); //!< The type of the constructor call

    /*!
     * Delayed constructor call of T class
     * 
     * \warning passing references or pointers as parameters means these objects will be given to the constructor at evaluation time.
     * Make sure these references/pointers are not invalidated between construction of the lazy object and the evaluation.
     */
    LazyInitialization(Args&&... args)
    : std::optional<T>()
    , bind_([&](Args&&... args) { std::optional<T>::instance = new T(args...); }, std::forward<Args>(args)...)
    { }

    /*!
     * Delayed function call for creating a T object
     * 
     * \warning passing references or pointers as parameters means these objects will be given to the function object at evaluation time.
     * Make sure these references/pointers are not invalidated between construction of the lazy object and the evaluation.
     */
    LazyInitialization(std::function<T (Args&&...)> f, Args&&... args)
    : std::optional<T>()
    , bind_([&](Args&&...) { std::optional<T>::instance = new T(f(args...)); }, std::forward<Args>(args)...)
    { }

    /*!
     * Delayed function call for creating a T object
     * 
     * \warning passing references or pointers as parameters means these objects will be given to the function object at evaluation time.
     * Make sure these references/pointers are not invalidated between construction of the lazy object and the evaluation.
     */
    LazyInitialization(std::function<T* (Args&&...)> f, Args&&... args)
    : std::optional<T>()
    , bind_([&](Args&&...) { std::optional<T>::instance = f(args...); }, std::forward<Args>(args)...)
    { }

    LazyInitialization(LazyInitialization<T, Args...>& other) //!< copy constructor
    : std::optional<T>(other)
    , bind_(other.bind_)
    { }

    LazyInitialization(LazyInitialization<T, Args...>&& other) //!< move constructor
    : std::optional<T>(other)
    {
        bind_ = std::move(other);
    }

    /*!
     * Dereference this lazy object
     * 
     * Calls constructor if object isn't constructed yet.
     */
    T& operator*()
    {
        if (!std::optional<T>::instance)
        {
            bind_();
        }
        return std::optional<T>::operator*();
    }

    T* operator->() const
    {
        if (!std::optional<T>::instance)
        {
            bind_();
        }
        return std::optional<T>::operator->();
    }

    LazyInitialization<T, Args...>& operator=(LazyInitialization<T, Args...>&& other)
    {
        std::optional<T>::operator=(other);
        bind_ = other.bind_;
        return *this;
    }

    void swap(LazyInitialization<T, Args...>& other)
    {
        std::optional<T>::swap(other);
        std::swap(bind_, other.bind_);
    }

private:
    bind_type bind_;
};

}//namespace cura
#endif // UTILS_LAZY_INITIALIZATION_H
