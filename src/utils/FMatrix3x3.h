//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef FMATRIX3X3_H
#define FMATRIX3X3_H

namespace cura
{

class Point3;
class FPoint3;

class FMatrix3x3
{
public:
    double m[3][3];

    FMatrix3x3();
    
    Point3 apply(const FPoint3& p) const;
};

} //namespace cura
#endif //FMATRIX3X3_H