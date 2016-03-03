#include "commandSocket.h"

namespace cura {

std::unique_ptr<CommandSocket> CommandSocket::instance_ = std::unique_ptr<CommandSocket>(new CommandSocket);

CommandSocket* CommandSocket::getInstance()
{
    return instance_.get();
}

void CommandSocket::setInstance(std::unique_ptr<CommandSocket> instance) {
    instance_ = std::move(instance);
}


}//namespace cura
