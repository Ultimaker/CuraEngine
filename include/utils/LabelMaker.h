// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef LABEL_MAKER_H
#define LABEL_MAKER_H

#include <string_view>
#include <vector>

namespace cura
{
void paintStringToBuffer(const std::string_view& str, const size_t buffer_width, const size_t buffer_height, std::vector<uint8_t>& buffer);
}

#endif // LABEL_MAKER_H
