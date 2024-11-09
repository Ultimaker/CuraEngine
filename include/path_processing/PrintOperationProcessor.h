// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef PATHPROCESSING_PRINTOPERATIONPROCESSOR_H
#define PATHPROCESSING_PRINTOPERATIONPROCESSOR_H

namespace cura
{

class PrintOperation;

template<class OperationType = PrintOperation>
class PrintOperationProcessor
{
public:
    virtual ~PrintOperationProcessor() = default; // Force class being polymorphic

    virtual void process(OperationType* operation) = 0;
};

} // namespace cura

#endif // PATHPROCESSING_PRINTOPERATIONPROCESSOR_H
