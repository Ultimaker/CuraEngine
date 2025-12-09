// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/MeshUtils.h"

#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <png.h>
#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/memorystream.h>

#include <spdlog/spdlog.h>

#include "TextureDataMapping.h"
#include "mesh.h"
#include "utils/Point2F.h"
#include "utils/Point3D.h"


namespace cura::MeshUtils
{

std::optional<Point3D> getBarycentricCoordinates(const Point3D& point, const Triangle3D& triangle)
{
    // Calculate vectors from p0 to p1 and p0 to p2
    const Point3D v0(triangle[1] - triangle[0]);
    const Point3D v1(triangle[2] - triangle[0]);
    const Point3D v2(point - triangle[0]);

    // Compute dot products
    const double d00 = v0 * v0;
    const double d01 = v0 * v1;
    const double d11 = v1 * v1;
    const double d20 = v2 * v0;
    const double d21 = v2 * v1;

    // Calculate denominator for barycentric coordinates
    const double denom = d00 * d11 - d01 * d01;

    // Check if triangle is degenerate
    constexpr double epsilon_triangle_cross_products = 0.000001;
    if (std::abs(denom) < epsilon_triangle_cross_products)
    {
        return std::nullopt;
    }

    // Calculate barycentric coordinates
    const double v = (d11 * d20 - d01 * d21) / denom;
    const double w = (d00 * d21 - d01 * d20) / denom;
    const double u = 1.0 - v - w;

    // Return as a Point3D where x/y/z represent the barycentric coordinates u/v/w
    return Point3D(u, v, w);
}

Point2F getUVCoordinates(const Point3D& barycentric_coordinates, const Triangle2F& uv_coordinates)
{
    return Point2F(
        (uv_coordinates[0].x_ * barycentric_coordinates.x_) + (uv_coordinates[1].x_ * barycentric_coordinates.y_) + (uv_coordinates[2].x_ * barycentric_coordinates.z_),
        (uv_coordinates[0].y_ * barycentric_coordinates.x_) + (uv_coordinates[1].y_ * barycentric_coordinates.y_) + (uv_coordinates[2].y_ * barycentric_coordinates.z_));
}

bool loadTextureFromPngData(const std::vector<unsigned char>& texture_data, Mesh& mesh, const std::string& source_description, bool log_no_metadata)
{
    if (texture_data.empty())
    {
        return false;
    }

    // Use PNG library to parse the texture
    png_image raw_texture = {};
    raw_texture.version = PNG_IMAGE_VERSION;
    if (! png_image_begin_read_from_memory(&raw_texture, texture_data.data(), texture_data.size()))
    {
        if (log_no_metadata)
        {
            spdlog::warn("Error reading PNG texture {}: {}", source_description, raw_texture.message);
        }
        else
        {
            spdlog::error("Error when beginning reading mesh texture: {}", raw_texture.message);
        }
        return false;
    }

    std::vector<uint8_t> buffer(PNG_IMAGE_SIZE(raw_texture));
    if (! png_image_finish_read(&raw_texture, nullptr, buffer.data(), 0, nullptr) || buffer.empty())
    {
        if (log_no_metadata)
        {
            spdlog::warn("Error finishing PNG texture read {}: {}", source_description, raw_texture.message);
        }
        else
        {
            spdlog::error("Error when finishing reading mesh texture: {}", raw_texture.message);
        }
        return false;
    }

    // Create PNG reading structures to extract metadata
    std::unique_ptr<png_struct, void (*)(png_structp)> png_ptr(
        png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr),
        [](png_structp png_ptr_destroy)
        {
            png_destroy_read_struct(&png_ptr_destroy, nullptr, nullptr);
        });
    if (! png_ptr)
    {
        return false;
    }

    std::unique_ptr<png_info, void (*)(png_infop)> info_ptr(
        png_create_info_struct(png_ptr.get()),
        [](png_infop info_ptr_destroy)
        {
            png_destroy_read_struct(nullptr, &info_ptr_destroy, nullptr);
        });
    if (! info_ptr)
    {
        return false;
    }

    if (setjmp(png_jmpbuf(png_ptr.get())) != 0)
    {
        return false;
    }

    struct PngReadContext
    {
        const unsigned char* data;
        size_t size;
        size_t offset;
    } read_context{ texture_data.data(), texture_data.size(), 0 };

    png_set_read_fn(
        png_ptr.get(),
        &read_context,
        [](const png_structp read_png_ptr, const png_bytep out_bytes, const png_size_t byte_count_to_read)
        {
            auto* context = static_cast<PngReadContext*>(png_get_io_ptr(read_png_ptr));
            if (context->offset + byte_count_to_read > context->size)
            {
                png_error(read_png_ptr, "Read beyond end of buffer");
            }
            memcpy(out_bytes, context->data + context->offset, byte_count_to_read);
            context->offset += byte_count_to_read;
        });
    png_read_info(png_ptr.get(), info_ptr.get());

    // Extract metadata from PNG text fields
    png_textp text_ptr;
    int num_text;
    if (png_get_text(png_ptr.get(), info_ptr.get(), &text_ptr, &num_text) <= 0)
    {
        if (log_no_metadata)
        {
            spdlog::info("No metadata found in texture: {}", source_description);
        }
        return false;
    }

    auto texture_data_mapping = std::make_shared<TextureDataMapping>();
    for (int i = 0; i < num_text; ++i)
    {
        if (std::string(text_ptr[i].key) == "Description")
        {
            rapidjson::MemoryStream json_memory_stream(text_ptr[i].text, text_ptr[i].text_length);

            rapidjson::Document json_document;
            json_document.ParseStream(json_memory_stream);
            if (json_document.HasParseError())
            {
                if (log_no_metadata)
                {
                    spdlog::warn(
                        "Error parsing texture metadata in {} (offset {}): {}",
                        source_description,
                        json_document.GetErrorOffset(),
                        GetParseError_En(json_document.GetParseError()));
                }
                else
                {
                    spdlog::error("Error parsing texture data mapping (offset {}): {}", json_document.GetErrorOffset(), GetParseError_En(json_document.GetParseError()));
                }
                return false;
            }

            // Parse the paint feature manifest
            for (auto it = json_document.MemberBegin(); it != json_document.MemberEnd(); ++it)
            {
                std::string feature_name = it->name.GetString();

                const rapidjson::Value& array = it->value;
                if (array.IsArray() && array.Size() == 2)
                {
                    (*texture_data_mapping)[feature_name] = TextureBitField{ array[0].GetUint(), array[1].GetUint() };
                }
            }

            break;
        }
    }

    if (! texture_data_mapping->empty())
    {
        mesh.texture_ = std::make_shared<Image>(
            raw_texture.width,
            raw_texture.height,
            PNG_IMAGE_SAMPLE_COMPONENT_SIZE(raw_texture.format) * PNG_IMAGE_SAMPLE_CHANNELS(raw_texture.format),
            std::move(buffer));
        mesh.texture_data_mapping_ = texture_data_mapping;

        if (log_no_metadata)
        {
            spdlog::info("Loaded texture {} with {} paint features", source_description, texture_data_mapping->size());
        }
        return true;
    }

    return false;
}

bool loadTextureFromFile(Mesh& mesh, const std::string& texture_filename)
{
    if (! std::filesystem::exists(texture_filename))
    {
        return false; // File doesn't exist, not an error
    }

    // Read PNG file into memory
    std::ifstream file(texture_filename, std::ios::binary | std::ios::ate);
    if (! file.is_open())
    {
        spdlog::warn("Failed to open texture file: {}", texture_filename);
        return false;
    }

    const std::streamsize file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<unsigned char> file_data(file_size);
    if (! file.read(reinterpret_cast<char*>(file_data.data()), file_size))
    {
        spdlog::warn("Failed to read texture file: {}", texture_filename);
        return false;
    }

    return loadTextureFromPngData(file_data, mesh, texture_filename, true);
}

void loadTextureFromString(const std::string& texture_str, Mesh& mesh)
{
    if (texture_str.empty())
    {
        return;
    }

    std::vector<unsigned char> texture_data(texture_str.begin(), texture_str.end());
    loadTextureFromPngData(texture_data, mesh, "network data", false);
}

} // namespace cura::MeshUtils
