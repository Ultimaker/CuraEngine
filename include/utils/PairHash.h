// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_PAIRHASH_H
#define UTILS_PAIRHASH_H

#include <bitset>
#include <utility>

namespace std
{

template<typename S, typename T>
struct hash<std::pair<S, T>>
{
    size_t operator()(const std::pair<S, T>& pair) const
    {
        return 31 * std::hash<S>()(pair.first) + 59 * std::hash<T>()(pair.second);
    }
};

} // namespace std

#endif // UTILS_PAIRHASH_H
