//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SUPPORTTEST_H
#define SUPPORTTEST_H

#include <cppunit/TestFixture.h>
#include "../src/sliceDataStorage.h"

namespace cura
{

class SupportTest : public CppUnit::TestFixture
{
public:
    /*!
     * \brief Creates the fixtures to test support with.
     */
    void setup();

    /*!
     * \brief Destroys the fixtures to test support with, freeing memory.
     */
    void tearDown();

private:
    
};

}

#endif //SUPPORTTEST_H