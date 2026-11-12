// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ENABLE_PLUGINS
#include "plugins/empty.h"

namespace curaengine::plugins
{
empty::value_type empty::operator()() const
{
    return {};
}

constexpr empty::native_value_type empty::operator()(const value_type&) const
{
    return nullptr;
}
} // namespace curaengine::plugins
#endif // ENABLE_PLUGINS
