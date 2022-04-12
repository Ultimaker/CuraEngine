//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef LITESUPPORT_H
#define LITESUPPORT_H

namespace cura
{
    class SliceDataStorage;
    class SliceMeshStorage;

    /*!
     * \brief Generates a 'lightning' structure to support models where needed.
     */
    class LightningSupport
    {
    public:
        /*!
         * \brief Creates an instance of the lightning support generator.
         * \param storage The data storage to get global settings from.
         */
        LightningSupport();

        /*!
         * \brief Create the areas that need support.
         *
         * These areas are stored inside the given SliceDataStorage object.
         * \param storage The data storage where the mesh data is gotten from and where the resulting support areas are stored.
         */
        void generateSupportAreas(SliceDataStorage& storage);

    protected:
        void generateSupportForMesh(SliceMeshStorage& mesh);
    }; // class LightningSupport

} // namespace cura

#endif //LITESUPPORT_H
