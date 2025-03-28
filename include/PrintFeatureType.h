// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

#include <cstdint>

namespace cura
{

enum class PrintFeatureType : uint8_t
{
    NoneType = 0, // used to mark unspecified jumps in polygons. libArcus depends on it
    OuterWall = 1,
    InnerWall = 2,
    Skin = 3,
    Support = 4,
    SkirtBrim = 5,
    Infill = 6,
    SupportInfill = 7,
    MoveCombing = 8,
    MoveRetraction = 9,
    SupportInterface = 10,
    PrimeTower = 11,
    Roof = 12,
    DraftShield = 13
    // When adding new types, make sure it still fits into the PrintFeatureMask inner type
};

class PrintFeatureMask
{
public:
    using mask_type = uint16_t;

    constexpr PrintFeatureMask(const PrintFeatureType type)
        : mask_(static_cast<mask_type>(1) << static_cast<mask_type>(type))
    {
    }

    constexpr explicit PrintFeatureMask(const mask_type mask)
        : mask_(mask)
    {
    }

    constexpr PrintFeatureMask operator|(const PrintFeatureMask& other) const
    {
        return PrintFeatureMask(mask_ | other.mask_);
    }

    bool hasType(const PrintFeatureType& type) const
    {
        return (mask_ & static_cast<mask_type>(type)) != 0;
    }

private:
    mask_type mask_;
};

namespace PrintFeatureTypes
{
static constexpr PrintFeatureMask model = PrintFeatureMask(PrintFeatureType::InnerWall) | PrintFeatureMask(PrintFeatureType::OuterWall) | PrintFeatureMask(PrintFeatureType::Skin)
                                        | PrintFeatureMask(PrintFeatureType::Infill) | PrintFeatureMask(PrintFeatureType::Roof);

static constexpr PrintFeatureMask support
    = PrintFeatureMask(PrintFeatureType::Support) | PrintFeatureMask(PrintFeatureType::SupportInfill) | PrintFeatureMask(PrintFeatureType::SupportInterface);

static bool isModel(const PrintFeatureType feature_type)
{
    return model.hasType(feature_type);
}
} // namespace PrintFeatureTypes

} // namespace cura
