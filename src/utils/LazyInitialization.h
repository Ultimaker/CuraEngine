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

    /*!
     * Delayed constructor call of T class
     *
     * \warning passing references or pointers as parameters means these objects will be given to the constructor at evaluation time.
     * Make sure these references/pointers are not invalidated between construction of the lazy object and the evaluation.
     */
    LazyInitialization(Args... args)
    : std::optional<T>()
    , constructor(
            [args...]()
            {
                return new T(args...);
            }
        )
    { }

    /*!
     * Delayed function call for creating a T object
     *
     * Performs a copy from the return value of the function on the stack to the heap.
     *
     * \warning passing references or pointers as parameters means these objects will be given to the function object at evaluation time.
     * Make sure these references/pointers are not invalidated between construction of the lazy object and the evaluation.
     */
    LazyInitialization(const std::function<T (Args...)>& f, Args... args)
    : std::optional<T>()
    , constructor(
            [f, args...]()
            {
                return new T(f(args...));
            }
        )
    { }

    /*!
     * Delayed function call for creating a T object
     *
     * \warning passing references or pointers as parameters means these objects will be given to the function object at evaluation time.
     * Make sure these references/pointers are not invalidated between construction of the lazy object and the evaluation.
     */
    LazyInitialization(const std::function<T* (Args...)>& f, Args... args)
    : std::optional<T>()
    , constructor(
            [f, args...]()
            {
                return f(args...);
            }
        )
    {
    }

    LazyInitialization(LazyInitialization<T, Args...>& other) //!< copy constructor
    : std::optional<T>(other)
    , constructor(other.constructor)
    {
    }

    LazyInitialization(LazyInitialization<T, Args...>&& other) //!< move constructor
    : std::optional<T>(other)
    {
        constructor = std::move(other.constructor);
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
            std::optional<T>::instance = constructor();
        }
        return std::optional<T>::operator*();
    }

    T* operator->() const
    {
        if (!std::optional<T>::instance)
        {
            std::optional<T>::instance = constructor();
        }
        return std::optional<T>::operator->();
    }

    LazyInitialization<T, Args...>& operator=(LazyInitialization<T, Args...>&& other)
    {
        std::optional<T>::operator=(other);
        constructor = other.constructor;
        return *this;
    }

    void swap(LazyInitialization<T, Args...>& other)
    {
        std::optional<T>::swap(other);
        std::swap(constructor, other.constructor);
    }

private:
    std::function<T* ()> constructor;
};

}//namespace cura
#endif // UTILS_LAZY_INITIALIZATION_H
