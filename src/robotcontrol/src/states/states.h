#ifndef STATES_H
#define STATES_H

#include "state.h"
#include "manualControlState.cpp"
#include "autonomousControlState.cpp"

static ManualControlState* manualControlState = new ManualControlState();
static AutonomousControlState* autonomousControlState = new AutonomousControlState();

#endif