// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#pragma once

namespace cura
{

class PrintOperation;
class SliceDataStorage;

template<class OperationType = PrintOperation>
class PrintOperationTransformer
{
public:
    virtual ~PrintOperationTransformer() = default; // Force class being polymorphic

    virtual void process(OperationType* operation) = 0;
};

} // namespace cura
