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

std::optional<uint32_t> TextureDataProvider::getValue(const Point2LL& position, const std::string& feature) const
{
    auto data_mapping_iterator = texture_data_mapping_->find(feature);
    if (data_mapping_iterator == texture_data_mapping_->end())
    {
        return std::nullopt;
    }

    const TextureBitField& bit_field = data_mapping_iterator->second;
    const std::optional<Point2F> point_uv_coordinates = uv_coordinates_->getClosestUVCoordinates(position);
    if (! point_uv_coordinates.has_value())
    {
        return std::nullopt;
    }

    const uint32_t pixel_data = texture_->getPixel(point_uv_coordinates.value());

    // Extract relevant bits by rotating the pixel data left then right, which will insert 0s where appropriate
    return (pixel_data << (32 - 1 - bit_field.bit_range_end_index)) >> (32 - 1 - (bit_field.bit_range_end_index - bit_field.bit_range_start_index));
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
