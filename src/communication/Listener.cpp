// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ARCUS

#include "communication/Listener.h"

#include <Arcus/Error.h> //To process error codes.
#include <spdlog/spdlog.h>

namespace cura
{

void Listener::stateChanged(Arcus::SocketState)
{
    // Do nothing.
}

void Listener::messageReceived()
{
    // Do nothing.
}

void Listener::error(const Arcus::Error& error)
{
    if (error.getErrorCode() == Arcus::ErrorCode::Debug)
    {
        spdlog::debug("{}", error.getErrorMessage());
    }
    else
    {
        spdlog::error("{}", error.getErrorMessage());
    }
}

} // namespace cura

#endif // ARCUS