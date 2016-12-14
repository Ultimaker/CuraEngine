/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef TEXTURE_PROCESSING_MATERIAL_BASE_H
#define TEXTURE_PROCESSING_MATERIAL_BASE_H

#include <unordered_map>
#include <string>
#include <vector>

#include "Material.h"

namespace cura
{

class MaterialBase
{
public:
    int getMatId(std::string name) const;
    Material* add(std::string name);
    const Material* getMat(unsigned int id) const;
protected:
    std::unordered_map<std::string, int> name_to_mat_idx;
    std::vector<Material> materials;
};

} // namespace cura

#endif // TEXTURE_PROCESSING_MATERIAL_BASE_H