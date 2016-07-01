/** Copyright (C) 2016 Scott Lenser - Released under terms of the AGPLv3 License */

#ifndef INCLUDED_WIRE_PLAN_SENDER_HPP
#define INCLUDED_WIRE_PLAN_SENDER_HPP

#include "BuildSubPlanner.hpp"
#include "GuiSocket.hpp"

namespace mason {

/** \brief Sends the wire plan to the GUI for display if connected. */
class WirePlanSender : public BuildSubPlanner {
public:
    virtual ~WirePlanSender() {}

    virtual void process(BuildPlan *build_plan);

private:
    void sendLayer(GuiSocket *socket, const WireLayer &wire_layer);

    int m_gui_layer_idx;
};

}

#endif
