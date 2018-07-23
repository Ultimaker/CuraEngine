//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef APPLICATION_H
#define APPLICATION_H

namespace cura
{

/*!
 * A singleton class that serves as the starting point for all slicing.
 *
 * The application provides a starting point for the slicing engine. It
 * maintains communication with other applications and uses that to schedule
 * slices.
 */
class Application
{
public:
    /*!
     * Gets the instance of this application class.
     */
    static Application& getInstance();

    /*!
     * Starts the application.
     *
     * It will start by parsing the command line arguments to see what it must
     * be doing.
     * \param argc The number of arguments provided to the application.
     * \param argv The arguments provided to the application.
     */
    void run(const int argc, char** argv);

protected:
#ifdef ARCUS
    /*!
     * Connect using libArcus to a socket.
     * \param argc The number of arguments provided to the application.
     * \param argv The arguments provided to the application.
     */
    void connect(const int argc, char** argv);
#endif //ARCUS

    /*!
     * Print to the stderr channel what the original call to the executable was.
     * \param argc The number of arguments provided to the application.
     * \param argv The arguments provided to the application.
     */
    void printCall(const int argc, char** argv) const;

    /*!
     * Print to the stderr channel how to use CuraEngine.
     */
    void printHelp() const;

    /*!
     * Print the header and license to the stderr channel.
     */
    void printLicense() const;

    /*!
     * Start slicing.
     * \param argc The number of arguments provided to the application.
     * \param argv The arguments provided to the application.
     */
    void slice(const int argc, char** argv);
};

} //Cura namespace.

#endif //APPLICATION_H