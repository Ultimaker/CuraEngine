// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "TextureDataProvider.h"

#include <memory>

#include <gtest/gtest.h>

#include "TextureDataMapping.h"
#include "mesh.h"

namespace cura
{

// getValue() on a 1x1, 4-bytes-per-pixel texture holding the pixel 0xABCD1234 (big-endian byte order,
// matching Image::getPixel), for the given (untrusted) bit field.
static std::optional<uint32_t> getValueFor(const TextureBitField& bit_field)
{
    auto texture = std::make_shared<Image>(1, 1, 4, std::vector<uint8_t>{ 0xABu, 0xCDu, 0x12u, 0x34u });
    auto mapping = std::make_shared<TextureDataMapping>();
    (*mapping)["feature"] = bit_field;
    return TextureDataProvider(nullptr, texture, mapping).getValue(0, 0, "feature");
}

// A valid, in-range bit field is accepted by the invariant and extracts exactly the requested bits, right-aligned.
TEST(TextureDataProviderTest, ValidBitFieldExtractsBits)
{
    EXPECT_TRUE((TextureBitField{ 0, 7 }.isValid()));
    EXPECT_EQ(getValueFor(TextureBitField{ 0, 7 }), std::optional<uint32_t>{ 0x34u }); // low byte
    EXPECT_EQ(getValueFor(TextureBitField{ 8, 15 }), std::optional<uint32_t>{ 0x12u }); // second byte
    EXPECT_EQ(getValueFor(TextureBitField{ 24, 31 }), std::optional<uint32_t>{ 0xABu }); // high byte
}

// Out-of-range / inverted ranges come from untrusted texture metadata. isValid() is the gate the PNG-metadata parser applies
// at the trust boundary (so the field never enters the mapping); getValue() rejects them too, so no >= 32-bit shift (UB) can run.
TEST(TextureDataProviderTest, OutOfRangeBitFieldIsRejected)
{
    for (const TextureBitField& bad : { TextureBitField{ 0, 32 }, TextureBitField{ 0, 64 }, TextureBitField{ 10, 5 } })
    {
        EXPECT_FALSE(bad.isValid());
        EXPECT_FALSE(getValueFor(bad).has_value());
    }
}

} // namespace cura
