// Copyright (c) 2025 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "TextureDataProvider.h"

#include "SlicedUVCoordinates.h"
#include "mesh.h"

namespace cura
{

TextureDataProvider::TextureDataProvider(
    const std::shared_ptr<SlicedUVCoordinates>& uv_coordinates,
    const std::shared_ptr<Image>& texture,
    const std::shared_ptr<TextureDataMapping>& texture_data_mapping)
    : uv_coordinates_(uv_coordinates)
    , texture_(texture)
    , texture_data_mapping_(texture_data_mapping)
{
}

std::optional<uint32_t> TextureDataProvider::getValue(const size_t pixel_x, const size_t pixel_y, const std::string& feature) const
{
    auto data_mapping_iterator = texture_data_mapping_->find(feature);
    if (data_mapping_iterator == texture_data_mapping_->end())
    {
        return std::nullopt;
    }

    const TextureBitField& bit_field = data_mapping_iterator->second;
    const uint32_t pixel_data = texture_->getPixel(pixel_x, pixel_y);

    // Extract relevant bits by rotating the pixel data left then right, which will insert 0s where appropriate
    return (pixel_data << (32 - 1 - bit_field.bit_range_end_index)) >> (32 - 1 - (bit_field.bit_range_end_index - bit_field.bit_range_start_index));
}

std::optional<uint32_t> TextureDataProvider::getValue(const Point2F& uv_coordinates, const std::string& feature) const
{
    std::pair<size_t, size_t> pixel_coordinates = texture_->getPixelCoordinates(uv_coordinates);
    return getValue(pixel_coordinates.first, pixel_coordinates.second, feature);
}

std::optional<uint32_t> TextureDataProvider::getValue(const Point2LL& position, const std::string& feature) const
{
    const std::optional<Point2F> point_uv_coordinates = uv_coordinates_->getClosestUVCoordinates(position);
    if (! point_uv_coordinates.has_value())
    {
        return std::nullopt;
    }

    return getValue(point_uv_coordinates.value(), feature);
}

std::optional<TextureArea> TextureDataProvider::getAreaPreference(const Point2LL& position, const std::string& feature) const
{
    const std::optional<uint32_t> raw_value = getValue(position, feature);
    if (raw_value.has_value())
    {
        return static_cast<TextureArea>(raw_value.value());
    }

    return std::nullopt;
}

} // namespace cura
