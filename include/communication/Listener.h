// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once
#ifdef ARCUS // Extends from Arcus::SocketListener, so only compile if we're using libArcus.

#include <Arcus/SocketListener.h> //The class we're extending from.

namespace curaengine::communication
{

/*
 * Extension of Arcus' ``SocketListener`` class to specialise the message
 * handling for CuraEngine.
 */
class Listener : public Arcus::SocketListener
{
public:
    /*
     * Changes the ``stateChanged`` signal to do nothing.
     */
    void stateChanged(Arcus::SocketState) override;

    /*
     * Changes the ``messageReceived`` signal to do nothing.
     */
    void messageReceived() override;

    /*
     * Log an error when we get one from libArcus.
     */
    void error(const Arcus::Error& error) override;
};

} // namespace curaengine::communication

#endif // ARCUS