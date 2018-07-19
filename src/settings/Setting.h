//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SETTING_H
#define SETTING_H

namespace cura
{

class ExtruderTrain; //Forward declaration to prevent having to include ExtruderTrain.h. Saves compilation time.

/*!
 * \brief Represents a single setting that is set to some value.
 */
struct Setting
{
    Setting(const std::string value, ExtruderTrain* limit_to_extruder = nullptr) : value(value), limit_to_extruder(limit_to_extruder) {};

    /*!
     * The value of the setting in it serialised form.
     * \todo Store this as a void pointer in its pre-parsed form so that getting
     * a setting doesn't need to parse it every time.
     */
    const std::string value;

    /*!
     * Unless overridden by a more specific setting container, limit the setting
     * to be used by a certain extruder.
     */
    ExtruderTrain* limit_to_extruder;
};

} //namespace cura

#endif //SETTING_H