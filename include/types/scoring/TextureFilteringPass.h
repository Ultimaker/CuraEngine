// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "types/geometry/Point2LL.h"
#include "types/scoring/FilteringPass.h"

namespace curaengine
{
class TextureDataProvider;
}

namespace curaengine::types::scoring
{

/*!
 * Pass that ranks the candidates according to the data contained in the user-painted texture, and keeps only the ones that have the highest
 * rank. The ranks, from the highest to the lowest, are: preferred area, normal area (or no texture data), avoided area.
 * The code is in the cpp file, so this class can only be used with the types of elements that are instantiated at the end of it.
 * \tparam T The type of the elements, which must have a position (see getElementPosition())
 */
template<typename T>
class TextureFilteringPass : public FilteringPass<T>
{
private:
    const std::shared_ptr<TextureDataProvider> texture_data_provider_;
    const std::string feature_name_;

public:
    explicit TextureFilteringPass(const std::shared_ptr<TextureDataProvider>& texture_data_provider, const std::string& feature_name);

protected:
    void filter(std::vector<Candidate<T>>& candidates) const override;

private:
    uint8_t computeRank(const types::geometry::Point2LL& position) const;
};

} // namespace curaengine::types::scoring
