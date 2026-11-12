// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifdef ARCUS

#include "communication/Listener.h"

#include <Arcus/Error.h>
#include <spdlog/spdlog.h>

namespace curaengine::communication
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

} // namespace curaengine::communication

#endif // ARCUS