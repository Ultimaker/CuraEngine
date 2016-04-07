/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef MATERIAL_BASE_H
#define MATERIAL_BASE_H

#include <unordered_map>
#include <string>
#include <vector>

#include "Material.h"

namespace cura
{

class MaterialBase
{
public:
    int getMatId(std::string name);
    Material* add(std::string name);
protected:
    std::unordered_map<std::string, int> name_to_mat_idx;
    std::vector<Material> materials;
};

} // namespace cura

#endif // MATERIAL_BASE_H