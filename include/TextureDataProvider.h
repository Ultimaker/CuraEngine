// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TEXTUREDATAPROVIDER_H
#define TEXTUREDATAPROVIDER_H

#include <memory>
#include <optional>

#include "TextureDataMapping.h"
#include "geometry/Point2LL.h"

namespace cura
{

class Image;
class SlicedUVCoordinates;
class Point2F;

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

    std::optional<uint32_t> getValue(const Point2F& uv_coordinates, const std::string& feature) const;

    std::optional<uint32_t> getValue(const Point2LL& position, const std::string& feature) const;

    std::optional<TextureArea> getAreaPreference(const Point2LL& position, const std::string& feature) const;

private:
    std::shared_ptr<SlicedUVCoordinates> uv_coordinates_;
    std::shared_ptr<Image> texture_;
    std::shared_ptr<TextureDataMapping> texture_data_mapping_;
};

} // namespace cura
#endif // TEXTUREDATAPROVIDER_H
