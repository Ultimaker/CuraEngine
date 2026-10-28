// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <memory>
#include <optional>

#include "TextureArea.h"
#include "TextureBitField.h"
#include "geometry/Point2LL.h"

namespace curaengine::geometry
{
class Point2F;
}

namespace curaengine
{

class Image;
class SlicedUVCoordinates;


class TextureDataProvider
{
public:
    explicit TextureDataProvider(
        const std::shared_ptr<SlicedUVCoordinates>& uv_coordinates,
        const std::shared_ptr<Image>& texture,
        const std::shared_ptr<TextureDataMapping>& texture_data_mapping);

    const std::shared_ptr<Image>& getTexture() const
    {
        return texture_;
    }

    std::optional<uint32_t> getValue(const size_t pixel_x, const size_t pixel_y, const std::string& feature) const;

    std::optional<uint32_t> getValue(const ::curaengine::geometry::Point2F& uv_coordinates, const std::string& feature) const;

    std::optional<uint32_t> getValue(const ::curaengine::geometry::Point2LL& position, const std::string& feature) const;

    std::optional<TextureArea> getAreaPreference(const ::curaengine::geometry::Point2LL& position, const std::string& feature) const;

private:
    std::shared_ptr<SlicedUVCoordinates> uv_coordinates_;
    std::shared_ptr<Image> texture_;
    std::shared_ptr<TextureDataMapping> texture_data_mapping_;
};

} // namespace curaengine
