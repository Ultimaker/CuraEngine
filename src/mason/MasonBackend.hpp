/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef INCLUDED_MASON_BACKEND_HPP
#define INCLUDED_MASON_BACKEND_HPP

#include "../MeshGroup.h"
#include "../gcodeExport.h"

namespace cura {
namespace mason {

class MasonBackend {
public:
   MasonBackend();

   void process(const MeshGroup *meshgroup, GCodeExport *gcode_out);
};

}
}

#endif
