// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef EMSCRIPTENCOMMUNICATION_H
#define EMSCRIPTENCOMMUNICATION_H
#ifdef __EMSCRIPTEN__

#include "communication/CommandLine.h"

namespace cura
{

/**
 * \class EmscriptenCommunication
 * \brief A class for handling communication in an Emscripten environment.
 *
 * This class extends the CommandLine class and provides specific implementations
 * for sending progress and handling slice information in an Emscripten environment.
 */
class EmscriptenCommunication : public CommandLine
{
private:
    std::string progress_handler_; ///< Handler for progress messages.
    std::string gcode_header_handler_; ///< Handler for getting the GCode handler.
    std::string slice_info_handler_; ///< Handler for slice information messages.
    std::string engine_info_handler_; ///< Handler for curaengine info : version and hash.
    /**
     * \brief Creates a message containing slice information.
     * \return A string containing the slice information message.
     */
    [[nodiscard]] static std::string createSliceInfoMessage();
    [[nodiscard]] static std::string createEngineInfoMessage();


public:
    /**
     * \brief Constructor for EmscriptenCommunication.
     * \param arguments A vector of strings containing the command line arguments.
     */
    EmscriptenCommunication(const std::vector<std::string>& arguments);

    /**
     * \brief Sends the progress of the current operation.
     * \param progress A double representing the progress percentage.
     */
    void sendProgress(double progress) const override;

    /**
     * \brief Sends GcodeHeader
     */
    void sendGCodePrefix(const std::string& prefix) const override;

    /**
     * \brief Indicate that we're beginning to send g-code.
     */
    void beginGCode() override;

    /**
     * \brief Initiates the slicing of the next item.
     */
    void sliceNext() override;

    bool isSequential() const override
    {
        return false;
    }
};

} // namespace cura

#endif // __EMSCRIPTEN__
#endif // EMSCRIPTENCOMMUNICATION_H
