/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_SYMMETRIC_PAIR
#define UTILS_SYMMETRIC_PAIR

#include <utility> // pair

namespace cura
{

/*!
 * A utility class for a pair of which the order between the first and the second doesn't matter.
 *
 * \tparam A The type of both elements of the pair.
 */
template<class A>
class SymmetricPair : public std::pair<A, A>
{
public:
    /*!
     * Forwarding std::pair constructor
     */
    template<class U>
    SymmetricPair(const SymmetricPair<U>& pr)
    : std::pair<A, A>(pr)
    {
    }
    /*!
     * Forwarding std::pair constructor
     */
    template<class U>
    SymmetricPair(SymmetricPair<U>&& pr)
    : std::pair<A, A>(pr)
    {
    }
    /*!
     * Forwarding std::pair constructor
     */
    SymmetricPair(const A& first, const A& second)
    : std::pair<A, A>(first, second)
    {
    }
    /*!
     * Forwarding std::pair constructor
     */
    template<class U>
    SymmetricPair(U&& first, U&& second)
    : std::pair<A, A>(first, second)
    {
    }
    /*!
     * Forwarding std::pair constructor
     */
    template <class... Args1, class... Args2>
    SymmetricPair(std::piecewise_construct_t pwc, std::tuple<Args1...> first_args, std::tuple<Args2...> second_args)
    : std::pair<A, A>(pwc, first_args, second_args)
    {
    }

    /*!
     * Equality operator which checks if two SymmetricPairs are equal regardless of the order between first and second
     */
    bool operator==(const SymmetricPair& other) const
    {
        return (std::pair<A, A>::first == other.first && std::pair<A, A>::second == other.second) || (std::pair<A, A>::first == other.second && std::pair<A, A>::second == other.first);
    }
};

}//namespace cura

namespace std
{

/*!
 * Hash operator which creates a hash regardless of the order between first and second
 */
template<class A>
struct hash<cura::SymmetricPair<A>>
{
    size_t operator()(const cura::SymmetricPair<A>& pr) const
    { // has to be symmetric wrt a and b!
        return std::hash<A>()(pr.first) + std::hash<A>()(pr.second);
    }
};
}//namespace std


#endif // UTILS_SYMMETRIC_PAIR