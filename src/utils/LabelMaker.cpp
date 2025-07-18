// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/LabelMaker.h"

#include <array>
#include <range/v3/view/enumerate.hpp>

// TODO: put 'font8x8' into config file instead
constexpr std::array<std::string_view, 16> font8x8 = {
R"(
..####..
.######.
##...###
##..#.##
##.#..##
###...##
.######.
..####..
)",
R"(
..####..
.#####..
....##..
....##..
....##..
....##..
.######.
.######.
)",
R"(
.######.
########
##....##
.....##.
....##..
..###...
.#######
########
)",
R"(
.######.
########
##.. ###
...###..
..####..
.... ###
########
.######.
)",
R"(
....###.
...#####
..##..##
.##...##
########
.#######
......##
......##
)",
R"(
.#######
########
##......
###.....
.######.
......##
.#######
######..
)",
R"(
..######
.#######
##......
######..
#######.
##....##
########
.#####..
)",
R"(
#######.
########
......##
.....##.
....##..
...##...
####....
####....
)",
R"(
..####..
.######.
##....##
.######.
.######.
##....##
.######.
..####..
)",
R"(
..#####.
########
##....##
.#######
..######
......##
#######.
######..
)",
R"(
...##...
..####..
.######.
##....##
########
########
##....##
##....##
)",
R"(
######..
########
##....##
#######.
#######.
##....##
########
######..
)",
R"(
..######
########
##......
##......
##......
##......
########
..######
)",
R"(
######..
########
##....##
##....##
##....##
##....##
########
######..
)",
R"(
.#######
########
##......
########
########
##......
########
.#######
)",
R"(
.#######
########
##......
########
########
##......
##......
##......
)",
};

std::vector<std::vector<bool>> makeFont(const std::array<std::string_view, 16>& font)
{
    std::vector<std::vector<bool>> res;
    for (const std::string_view& glyph : font)
    {
        res.emplace_back();
        for (const char c : glyph)
        {
            switch (c)
            {
            case '.': res.back().push_back(false); break;
            case '#': res.back().push_back(true); break;
            }
        }
    }
    return res;
}

void cura::paintStringToBuffer(const std::string_view& str, const size_t buffer_width, const size_t buffer_height, std::vector<uint8_t>& buffer)
{
    static auto proc_font = makeFont(font8x8);

    constexpr size_t base_offset_x = 3;
    size_t offset_x = base_offset_x;
    size_t offset_y = 3;
    for (char c : str)
    {
        char offset = 0;
        if (c >= 'a' && c <= 'z') { offset = 'a' - 0xA; }
        else if (c >= 'A' && c <= 'Z') { offset = 'A' - 0xA; }
        else if (c >= '0' && c <= '9') { offset = '0'; }
        else { c = -1; }
        c -= offset;

        if (c >= 0 && c < proc_font.size())
        {
            const auto& glyph = proc_font[c];
            for (const auto& [idx, pix] : glyph | ranges::views::enumerate)
            {
                const size_t x = offset_x + idx % 8;
                const size_t y = offset_y + idx / 8;
                buffer[y * buffer_width + x] = pix;
            }
        }

        offset_x += 10;
        if (offset_x + 1 >= buffer_width)
        {
            offset_x = base_offset_x;
            offset_y += 10;
        }
        if (offset_y + 1 >= buffer_height)
        {
            return;
        }
    }
}
