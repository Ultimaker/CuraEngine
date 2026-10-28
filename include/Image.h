// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "geometry/Point2F.h"

namespace curaengine
{
class Image
{
public:
    explicit Image() = default;
    explicit Image(const size_t width, const size_t height, const size_t bytes_per_pixel, std::vector<uint8_t>&& data)
        : width_(width)
        , height_(height)
        , bytes_per_pixel_(bytes_per_pixel)
        , bytes_per_row_(bytes_per_pixel * width)
        , data_(std::move(data))
    {
    }

    size_t getWidth() const
    {
        return width_;
    }

    size_t getHeight() const
    {
        return height_;
    }

    uint32_t getPixel(const std::pair<size_t, size_t>& pixel_coordinates) const
    {
        return getPixel(pixel_coordinates.first, pixel_coordinates.second);
    }

    uint32_t getPixel(const size_t x, const size_t y) const
    {
        uint32_t result = 0;
        const size_t index = y * bytes_per_row_ + x * bytes_per_pixel_;
        for (size_t i = 0; i < bytes_per_pixel_; ++i)
        {
            result |= (data_[index + i] << ((bytes_per_pixel_ - i - 1) * 8));
        }
        return result;
    }

    std::pair<size_t, size_t> getPixelCoordinates(const Point2F& uv_coordinates) const
    {
        return { std::clamp(static_cast<size_t>(uv_coordinates.x_ * width_), static_cast<size_t>(0), width_ - 1),
                 std::clamp(static_cast<size_t>(uv_coordinates.y_ * height_), static_cast<size_t>(0), height_ - 1) };
    }

    uint32_t getPixel(const Point2F& uv_coordinates) const
    {
        return getPixel(getPixelCoordinates(uv_coordinates));
    }

private:
    std::vector<uint8_t> data_; // The raw pixels, data
    size_t width_{ 0 }; // The image width
    size_t height_{ 0 }; // The image height
    size_t bytes_per_pixel_{ 0 }; // The number of bytes for each pixel
    size_t bytes_per_row_{ 0 };
};

} // namespace curaengine
