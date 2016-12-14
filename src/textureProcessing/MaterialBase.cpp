/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */

#include "MaterialBase.h"


namespace cura
{

Material* MaterialBase::add(std::string name)
{
    name_to_mat_idx[name] = materials.size();
    materials.emplace_back();
    return &materials.back();
}

const Material* MaterialBase::getMat(unsigned int id) const
{
    if (id < materials.size())
    {
        return &materials[id];
    }
    else
    {
        return nullptr;
    }
}


int MaterialBase::getMatId(std::string name) const
{
    auto it = name_to_mat_idx.find(name);
    if (it == name_to_mat_idx.end())
    {
        return -1;
    }
    else
    {
        return it->second;
    }
}

} // namespace cura